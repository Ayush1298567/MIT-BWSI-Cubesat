#!/usr/bin/env python3
# main.py — MuraltZ CubeSat flight software state machine
#
# State machine: BOOT → [wait for start_pass] → IMAGING → IDLE → DOWNLINK
#                           ↑______________wait for start_pass______________↑
#
# The loop does NOT auto-start the next pass. The operator must type "start_pass"
# to begin each imaging cycle. This gives them time to physically reposition the
# CubeSat over a new grid area and modify the surface for change detection.
#
# Operator terminal commands (typed into this process's stdin):
#   start_pass           — begin next imaging pass
#   end_pass             — stop current imaging pass early, proceed to IDLE
#   cell R C             — set current grid cell (e.g. "cell 2 3")
#   status               — print current system status
#   eclipse_on           — switch camera to low-light mode (100ms exposure, gain 8)
#   eclipse_off          — return camera to auto-exposure
#   kill_link            — suspend downlink (simulates link loss)
#   resume_link          — resume downlink attempts
#   resume               — exit safe mode
#   shutdown             — clean shutdown

import os
import queue as _queue_mod
import sys
import threading
import time

# Ensure the cubesat_flight package root is on the path when run directly.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from config import (
    DEFAULT_GRID_CELL,
    DOWNLINK_WINDOW_SEC,
    IMAGE_WIDTH,
    IMAGE_HEIGHT,
    GROUND_STATION_IP,
    PASS_CYCLE_DELAY_SEC,
    THROTTLE_BYTES_PER_SEC,
)
from comms.command_listener import CommandListener
from comms.transfer import Transfer, send_telemetry_now
from utils.telemetry import build_telemetry, build_state_telemetry
from processing.coverage import CoverageTracker
from processing.metadata import MetadataBuilder
from processing.quality import QualityGate
from sensors.camera import Camera
from sensors.imu import IMU
from states.boot import boot
from states.downlink import downlink
from states.idle import idle
from states.imaging import imaging_loop
from states.safe_mode import safe_mode
from storage.manager import StorageManager
from utils.logger import log, set_state
from utils.thermal import Thermal
from utils.watchdog import Watchdog


# ---------------------------------------------------------------------------
# Stdin reader — daemon thread that feeds operator commands into a queue
# so the main loop can pet the watchdog while waiting for start_pass.
# ---------------------------------------------------------------------------

_stdin_q: _queue_mod.Queue = _queue_mod.Queue()


def _start_stdin_reader():
    def _reader():
        while True:
            try:
                line = sys.stdin.readline()
                if not line:    # EOF
                    break
                stripped = line.strip().lower()
                if stripped:
                    _stdin_q.put(stripped)
            except (OSError, EOFError):
                break

    t = threading.Thread(target=_reader, daemon=True, name="StdinReader")
    t.start()


def _get_operator_input():
    """Non-blocking: return next operator line or None."""
    try:
        return _stdin_q.get_nowait()
    except _queue_mod.Empty:
        return None


def _push_state_telemetry(state, imu, thermal, storage, pass_number):
    """Send a lightweight state-transition telemetry packet to the GCS.
    Non-fatal: silently continues if GCS is unreachable.
    """
    telem = build_state_telemetry(state, imu, thermal, storage, pass_number)
    ok = send_telemetry_now(telem)
    log(f"State telemetry sent: {state}" if ok else f"State telemetry not delivered ({state}) — GCS down")


# ---------------------------------------------------------------------------
# Helper: handle GCS commands that affect global state
# ---------------------------------------------------------------------------

def _handle_gcs_command(cmd, camera, transfer, state_ref):
    """Apply a GCS command that is meaningful outside the imaging loop.

    Args:
        state_ref: dict with mutable flags: gcs_suspended, enter_safe
    """
    name = cmd.get("cmd")

    if name == "retry_downlink":
        state_ref["gcs_suspended"] = False
        transfer.consecutive_failures = 0
        log("GCS suspension lifted — downlink will resume next pass")

    elif name == "enter_safe_mode":
        log("GCS command: enter_safe_mode", level="WARN")
        state_ref["enter_safe"] = True

    elif name == "resume_normal":
        log("GCS command: resume_normal — already in normal operation")

    elif name == "adjust_exposure":
        exposure_us = cmd.get("exposure_us")
        if exposure_us:
            camera._cam.set_controls({"AeEnable": False, "ExposureTime": int(exposure_us)})
            log(f"Exposure set to {exposure_us} µs by GCS command")

    elif name == "status_request":
        # Cannot send telemetry here without a connection — log acknowledgement.
        log("GCS status_request received (telemetry sent at next downlink window)")

    elif name in ("retransmit", "priority_cell", "set_cell"):
        # These are only meaningful in IDLE/IMAGING — log and discard.
        log(f"GCS command '{name}' received outside IDLE — will apply next pass")

    else:
        log(f"Unknown GCS command: {name!r}", level="WARN")


# ---------------------------------------------------------------------------
# Wait-for-start-pass loop
# ---------------------------------------------------------------------------

def _wait_for_start_pass(command_listener, watchdog, camera, transfer, thermal, storage, state_ref, imu, pass_number):
    """Block until the operator types 'start_pass'.
    Pets the watchdog and handles GCS/operator commands while waiting.

    Returns True to begin pass, False to exit main loop (shutdown).
    """
    set_state("WAITING")
    _push_state_telemetry("WAITING", imu, thermal, storage, pass_number)
    log("Waiting for operator 'start_pass' command...")
    print("\n" + "=" * 60)
    print("READY FOR NEXT PASS")
    print("  start_pass  — begin imaging")
    print("  cell R C    — set grid cell (e.g. cell 2 3)")
    print("  status      — system status")
    print("  eclipse_on/off")
    print("  kill_link / resume_link")
    print("  shutdown    — exit")
    print("=" * 60)

    while True:
        watchdog.pet()

        # GCS commands while waiting.
        for cmd in command_listener.get_pending():
            name = cmd.get("cmd")
            if name == "start_pass":
                log("GCS start_pass received — beginning pass")
                return "start"
            elif name == "cell":
                try:
                    state_ref["current_grid_cell"] = (int(cmd["row"]), int(cmd["col"]))
                    log(f'Grid cell pre-set \u2192 {state_ref["current_grid_cell"]} (GCS)')
                except (KeyError, ValueError):
                    log("Malformed cell command from GCS", level="WARN")
            else:
                _handle_gcs_command(cmd, camera, transfer, state_ref)
                if state_ref.get("enter_safe"):
                    return "safe_mode"

        op = _get_operator_input()
        if not op:
            time.sleep(0.2)
            continue

        if op == "start_pass":
            return "start"

        elif op == "shutdown":
            log("Operator shutdown command received")
            return "shutdown"

        elif op == "status":
            cap = storage.check_capacity()
            print(
                f"  storage: {cap['used_pct']:.1f}% used ({cap['free_mb']:.0f} MB free)\n"
                f"  cpu_temp: {thermal.get_cpu_temp():.1f}°C\n"
                f"  gcs_suspended: {state_ref.get('gcs_suspended', False)}"
            )

        elif op == "eclipse_on":
            camera.set_low_light_mode()
            log("Eclipse mode ON (100ms exposure, gain 8.0)")

        elif op == "eclipse_off":
            camera.set_normal_mode()
            log("Eclipse mode OFF (auto-exposure)")

        elif op == "kill_link":
            state_ref["gcs_suspended"] = True
            log("Link killed by operator — downlink suspended")

        elif op == "resume_link":
            state_ref["gcs_suspended"] = False
            transfer.consecutive_failures = 0
            log("Link resumed by operator")

        elif op == "resume":
            log("'resume' typed during WAITING (already in normal mode)")

        elif op.startswith("cell"):
            # Allow cell pre-setting before start_pass.
            parts = op.split()
            try:
                state_ref["current_grid_cell"] = (int(parts[1]), int(parts[2]))
                log(f"Grid cell pre-set → {state_ref['current_grid_cell']}")
            except (IndexError, ValueError):
                print("  Bad cell command. Format: cell R C")

        else:
            print(f"  Unknown command: {op!r}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    # === Infrastructure — start these first so logging is available everywhere ===
    thermal = Thermal()
    thermal.start_monitoring()

    set_state("BOOT")
    log("=== MuraltZ CubeSat flight software starting ===")

    storage = StorageManager()
    storage.load_image_index()

    command_listener = CommandListener()
    command_listener.start()

    _start_stdin_reader()

    # === BOOT ===
    log("=== BOOT SELF-TEST ===")
    imu = IMU()
    camera = Camera(IMAGE_WIDTH, IMAGE_HEIGHT)

    boot_result = boot(imu, camera, storage, GROUND_STATION_IP)

    # Initialise remaining objects (before potential safe mode so GCS can recover).
    quality_gate = QualityGate()
    coverage = CoverageTracker()
    coverage.load()
    metadata_builder = MetadataBuilder()
    transfer = Transfer()

    # Mutable state — passed into helpers as a dict to allow mutation.
    state_ref = {
        "gcs_suspended": False,
        "enter_safe": False,
        "current_grid_cell": DEFAULT_GRID_CELL,
    }

    # Recovery file from previous run (watchdog restart or manual shutdown).
    recovery = boot_result.get("recovery_state")
    pass_number = recovery["pass_number"] if recovery else 0
    if recovery:
        state_ref["gcs_suspended"] = recovery.get("gcs_suspended", False)
        log(f"Resumed from recovery: pass={pass_number}, gcs_suspended={state_ref['gcs_suspended']}")

    # Running counters (reset on each start, not persisted — used for telemetry).
    captured_total = 0
    rejected_total = 0
    images_sent_total = 0

    # Start watchdog with a recovery callback that saves current pass number.
    def _recovery_cb():
        storage.save_recovery_state({
            "pass_number": pass_number,
            "gcs_suspended": state_ref["gcs_suspended"],
        })

    watchdog = Watchdog()
    watchdog.start(recovery_callback=_recovery_cb)

    # Handle boot failure — enter safe mode immediately.
    if not boot_result["success"]:
        log(f"Boot FAILED: {boot_result['reason']}", level="ERROR")
        set_state("SAFE_MODE")
        safe_mode(camera, command_listener, watchdog)
        # If safe_mode returns (resume_normal), try to continue.

    if boot_result.get("storage_only"):
        log("Storage-only mode: imaging disabled, will attempt downlink only", level="WARN")

    log(
        f"Boot complete. Pass counter={pass_number}. "
        f"GCS: {'reachable' if boot_result['gcs_reachable'] else 'UNREACHABLE — autonomous mode'}"
    )

    # =========================================================================
    # MAIN STATE MACHINE LOOP
    # =========================================================================

    while True:
        watchdog.pet()
        state_ref["enter_safe"] = False

        # -----------------------------------------------------------------
        # Wait for operator "start_pass"
        # -----------------------------------------------------------------
        action = _wait_for_start_pass(
            command_listener, watchdog, camera, transfer, thermal, storage, state_ref,
            imu=imu, pass_number=pass_number
        )

        if action == "shutdown":
            log("Shutting down cleanly")
            camera.close()
            storage.save_image_index()
            storage.save_recovery_state({"pass_number": pass_number, "gcs_suspended": state_ref["gcs_suspended"]})
            sys.exit(0)

        if action == "safe_mode":
            set_state("SAFE_MODE")
            safe_mode(camera, command_listener, watchdog)
            state_ref["enter_safe"] = False
            continue  # Back to waiting for start_pass

        # action == "start"
        pass_number += 1
        nadir_locked = False    # Reset each pass

        # -----------------------------------------------------------------
        # IMAGING
        # -----------------------------------------------------------------
        set_state("IMAGING")
        log(f"=== PASS {pass_number} — IMAGING ===")
        _push_state_telemetry("IMAGING", imu, thermal, storage, pass_number)

        if boot_result.get("storage_only"):
            log("Storage-only mode: skipping imaging", level="WARN")
            images_this_pass = []
            rejections = {"blur": 0, "underexposed": 0, "overexposed": 0, "motion_blur": 0}
            enter_safe = False
        else:
            (images_this_pass,
             rejections,
             state_ref["current_grid_cell"],
             nadir_locked,
             enter_safe) = imaging_loop(
                imu=imu,
                camera=camera,
                quality_gate=quality_gate,
                coverage=coverage,
                storage=storage,
                thermal=thermal,
                command_listener=command_listener,
                metadata_builder=metadata_builder,
                pass_number=pass_number,
                current_grid_cell=state_ref["current_grid_cell"],
                get_operator_input=_get_operator_input,
            )

        captured_this_pass = len(images_this_pass)
        captured_total += captured_this_pass
        rejected_this_pass = sum(rejections.values())
        rejected_total += rejected_this_pass

        log(
            f"Imaging complete: {captured_this_pass} captured, "
            f"{rejected_this_pass} rejected {rejections}"
        )

        if enter_safe:
            set_state("SAFE_MODE")
            safe_mode(camera, command_listener, watchdog)
            state_ref["enter_safe"] = False
            continue

        # -----------------------------------------------------------------
        # IDLE
        # -----------------------------------------------------------------
        set_state("IDLE")
        log(f"=== PASS {pass_number} — IDLE ===")
        _push_state_telemetry("IDLE", imu, thermal, storage, pass_number)

        queue = idle(
            storage=storage,
            coverage=coverage,
            command_listener=command_listener,
            pass_number=pass_number,
            camera=camera,
        )

        # Check if any deferred GCS command triggered safe mode.
        for cmd in command_listener.get_pending():
            _handle_gcs_command(cmd, camera, transfer, state_ref)
        if state_ref.get("enter_safe"):
            set_state("SAFE_MODE")
            safe_mode(camera, command_listener, watchdog)
            state_ref["enter_safe"] = False
            continue

        watchdog.pet()

        # -----------------------------------------------------------------
        # DOWNLINK
        # -----------------------------------------------------------------
        set_state("DOWNLINK")
        log(f"=== PASS {pass_number} — DOWNLINK ===")
        _push_state_telemetry("DOWNLINK", imu, thermal, storage, pass_number)

        bytes_sent_this_pass = 0
        images_sent_this_pass = 0

        if state_ref["gcs_suspended"]:
            log("GCS suspended — skipping downlink. Images queuing locally.")
        else:
            telem = build_telemetry(
                imu=imu,
                camera=camera,
                thermal=thermal,
                storage=storage,
                coverage=coverage,
                queue=queue,
                pass_number=pass_number,
                state="DOWNLINK",
                rejections=rejections,
                captured_this_pass=captured_this_pass,
                captured_total=captured_total,
                rejected_total=rejected_total,
                bytes_sent_this_pass=0,
                images_sent_total=images_sent_total,
                gcs_reachable=not state_ref["gcs_suspended"],
                nadir_locked=nadir_locked,
            )

            dl = downlink(
                transfer=transfer,
                storage=storage,
                queue=queue,
                telemetry_dict=telem,
                watchdog=watchdog,
            )

            queue = dl["remaining_queue"]
            bytes_sent_this_pass = dl["bytes_sent"]
            images_sent_this_pass = dl["images_sent"]
            images_sent_total += images_sent_this_pass

            if dl["gcs_suspended"]:
                state_ref["gcs_suspended"] = True
                log(
                    "GCS unreachable after 3 attempts — suspending downlink. "
                    "Images queuing locally.",
                    level="WARN",
                )

        # Check if GCS came back online via a command that arrived during downlink.
        for cmd in command_listener.get_pending():
            _handle_gcs_command(cmd, camera, transfer, state_ref)

        # -----------------------------------------------------------------
        # End-of-pass housekeeping
        # -----------------------------------------------------------------
        storage.save_queue(queue)
        storage.save_image_index()
        storage.save_recovery_state({
            "pass_number": pass_number,
            "gcs_suspended": state_ref["gcs_suspended"],
        })

        log(
            f"Pass {pass_number} complete: "
            f"{images_sent_this_pass} sent, {bytes_sent_this_pass} bytes, "
            f"{len(queue)} images still queued. "
            f"Total captured={captured_total}, sent={images_sent_total}."
        )

        watchdog.pet()
        time.sleep(PASS_CYCLE_DELAY_SEC)
        # Loop back → wait for next "start_pass"


if __name__ == "__main__":
    main()
