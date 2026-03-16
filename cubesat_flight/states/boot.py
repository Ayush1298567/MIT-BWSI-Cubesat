# states/boot.py — Boot self-test
#
# Runs at startup to verify all hardware is functional before entering the
# main imaging loop. Tests IMU, camera, storage, and GCS reachability.
#
# Failure policy:
#   IMU fail   → return failure (caller enters safe mode)
#   Camera fail → return failure (caller enters safe mode)
#   GCS fail   → log "autonomous mode", continue normally
#   Storage > 98% → return storage_only flag (caller goes downlink-only)

import os
import socket
import subprocess
import tempfile
import time

import cv2

from config import (
    BOOT_TIMEOUT,
    GROUND_STATION_IP,
    IMAGE_DIR,
    STORAGE_CRITICAL_PCT,
    DATA_PORT,
)
from utils.logger import log


def boot(imu, camera, storage, gcs_ip=GROUND_STATION_IP):
    """Run the boot self-test sequence.

    Args:
        imu:     Initialised IMU instance.
        camera:  Initialised Camera instance.
        storage: Initialised StorageManager instance.
        gcs_ip:  Ground station IP to ping.

    Returns:
        dict with keys:
            success        bool  — False means hardware failure, enter safe mode
            reason         str   — Human-readable result description
            recovery_state dict|None — Contents of recovery file if present
            gcs_reachable  bool  — Whether GCS responded
            storage_only   bool  — True if storage is critical (downlink-only mode)
    """
    start = time.monotonic()
    result = {
        "success": True,
        "reason": "Boot OK",
        "recovery_state": None,
        "gcs_reachable": False,
        "storage_only": False,
    }

    # --- 1. IMU self-test ---
    log("Boot: testing IMU...")
    try:
        ax, ay, az = imu.get_acceleration()
        import math as _math
        accel_mag = _math.sqrt(ax*ax + ay*ay + az*az)
        if not (8.0 < accel_mag < 35.0):
            result["success"] = False
            result["reason"] = (
                f"IMU self-test failed: |accel|={accel_mag:.2f} m/s2 "
                f"(expected 8.0-35.0, sensor unresponsive or disconnected)"
            )
            log(result["reason"], level="ERROR")
            return result
        log(f"Boot: IMU OK - accel=({ax:.2f}, {ay:.2f}, {az:.2f}) m/s2, |a|={accel_mag:.2f}")
    except Exception as exc:
        result["success"] = False
        result["reason"] = f"IMU exception during boot: {exc}"
        log(result["reason"], level="ERROR")
        return result

    if time.monotonic() - start > BOOT_TIMEOUT:
        result["success"] = False
        result["reason"] = "Boot timeout exceeded during IMU test"
        return result

    # --- 2. Camera self-test ---
    log("Boot: testing camera...")
    test_path = os.path.join(tempfile.gettempdir(), "cubesat_boot_test.jpg")
    try:
        camera.capture(test_path)
        size = os.path.getsize(test_path)
        if size < 1_000:
            result["success"] = False
            result["reason"] = f"Camera test image too small: {size} bytes (need > 1 KB)"
            log(result["reason"], level="ERROR")
            return result

        # Verify the file is a decodable JPEG.
        img = cv2.imread(test_path)
        if img is None:
            result["success"] = False
            result["reason"] = "Camera test image is not decodable by OpenCV"
            log(result["reason"], level="ERROR")
            return result

        log(f"Boot: camera OK — test image {size} bytes, shape {img.shape}")
    except Exception as exc:
        result["success"] = False
        result["reason"] = f"Camera exception during boot: {exc}"
        log(result["reason"], level="ERROR")
        return result
    finally:
        # Clean up test file; ignore errors.
        try:
            os.remove(test_path)
        except OSError:
            pass

    if time.monotonic() - start > BOOT_TIMEOUT:
        result["success"] = False
        result["reason"] = "Boot timeout exceeded during camera test"
        return result

    # --- 3. Storage check ---
    log("Boot: checking storage...")
    try:
        capacity = storage.check_capacity()
        log(f"Boot: storage — {capacity['used_pct']:.1f}% used, {capacity['free_mb']:.0f} MB free")
        if capacity["used_pct"] > STORAGE_CRITICAL_PCT:
            result["storage_only"] = True
            log(
                f"Storage critical ({capacity['used_pct']:.1f}%) — entering downlink-only mode",
                level="WARN",
            )
    except Exception as exc:
        log(f"Boot: storage check failed: {exc}", level="WARN")
        # Non-fatal — continue.

    # --- 4. GCS ping ---
    log(f"Boot: pinging GCS at {gcs_ip}...")
    result["gcs_reachable"] = _ping_gcs(gcs_ip)
    if result["gcs_reachable"]:
        log("Boot: GCS reachable")
    else:
        log("Boot: GCS unreachable — autonomous mode (non-fatal)", level="WARN")

    # --- 5. Recovery file ---
    try:
        recovery = storage.load_recovery_state()
        if recovery:
            log(f"Boot: recovery file found — resuming from pass {recovery.get('pass_number', 0)}")
            result["recovery_state"] = recovery
        else:
            log("Boot: no recovery file — clean start")
    except Exception as exc:
        log(f"Boot: could not load recovery state: {exc}", level="WARN")

    # --- Done ---
    elapsed = time.monotonic() - start
    log(f"Boot complete in {elapsed:.1f}s. Result: {result['reason']}")
    return result


def _ping_gcs(ip, timeout_sec=3):
    """Attempt a TCP connection to the GCS DATA_PORT.
    Returns True if the connection succeeds (GCS server is up),
    False if refused or timed out (GCS offline — non-fatal)."""
    try:
        with socket.create_connection((ip, DATA_PORT), timeout=timeout_sec):
            return True
    except OSError:
        return False
