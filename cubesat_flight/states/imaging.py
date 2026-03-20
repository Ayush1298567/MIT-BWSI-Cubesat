# states/imaging.py — IMU-gated capture loop
#
# imaging_loop() runs for up to IMAGING_WINDOW_SEC seconds (or until the
# operator types "end_pass"). It only captures when:
#   • imu.is_stable()  (angular rate < 1.0 rad/s)
#   • nadir_locked     (angle from -Z < 45°, using hysteresis to prevent toggling)
#
# Nadir hysteresis is maintained HERE, not in the IMU module:
#   Lock engages when angle < NADIR_TOLERANCE_DEG (45°)
#   Lock releases when angle > NADIR_EXIT_DEG (55°)
#
# Grid cell is set by operator terminal input ("cell R C") or by GCS "set_cell"
# command. The updated cell is returned to the caller so it persists across passes.

import os
import time
from datetime import datetime, timezone

from config import (
    CAPTURE_INTERVAL_SEC,
    IMAGE_DIR,
    IMAGING_WINDOW_SEC,
    JPEG_MAX_SIZE_BYTES,
    MAX_IMAGES_PER_PASS,
    NADIR_EXIT_DEG,
    NADIR_TOLERANCE_DEG,
    SCIENCE_WEIGHT_NOVELTY,
    SCIENCE_WEIGHT_QUALITY,
    SCIENCE_WEIGHT_REVISIT,
    SCIENCE_WEIGHT_TASK_MATCH,
)
from utils.logger import log


def imaging_loop(
    imu,
    camera,
    quality_gate,
    coverage,
    storage,
    thermal,
    command_listener,
    metadata_builder,
    pass_number,
    current_grid_cell,
    current_task,
    get_operator_input,
    watchdog=None,
):
    """Run one imaging pass.

    Args:
        imu, camera, quality_gate, coverage, storage, thermal: hardware/processing objects
        command_listener:   CommandListener for GCS commands
        metadata_builder:   MetadataBuilder instance
        pass_number:        Current pass number (int)
        current_grid_cell:  (row, col) tuple from operator — updated live during the pass
        get_operator_input: Callable() → str|None — non-blocking stdin line or None

    Returns:
        (images_this_pass, rejections, current_grid_cell, current_task, nadir_locked, enter_safe_mode)
        images_this_pass   list of metadata dicts for accepted images
        rejections         dict {"blur": N, "underexposed": N, "overexposed": N, "motion_blur": N}
        current_grid_cell  (row, col) — possibly updated by operator during the pass
        current_task       dict|None — active GCS mission task for scoring/prioritisation
        nadir_locked       bool — final nadir lock state (passed to telemetry)
        enter_safe_mode    bool — True if thermal critical triggered during imaging
    """
    images_this_pass = []
    rejections = {"blur": 0, "underexposed": 0, "overexposed": 0, "motion_blur": 0}
    nadir_locked = False    # Reset each pass — operator starts by holding CubeSat still
    end_pass = False        # Flag set by "end_pass" command; avoids break-from-inner-loop

    img_start = time.monotonic()

    os.makedirs(IMAGE_DIR, exist_ok=True)

    while not end_pass and (time.monotonic() - img_start) < IMAGING_WINDOW_SEC:
        if watchdog is not None:
            watchdog.pet()

        # --- Process GCS commands ---
        for cmd in command_listener.get_pending():
            cmd_name = cmd.get("cmd")
            if cmd_name in ("set_cell", "cell"):
                current_grid_cell = (cmd["row"], cmd["col"])
                log(f"Grid cell → {current_grid_cell} (GCS {cmd_name})")
            elif cmd_name in ("observe_cell", "revisit_cell"):
                current_task = {
                    "cmd": cmd_name,
                    "row": int(cmd["row"]),
                    "col": int(cmd["col"]),
                    "reason": cmd.get("reason", ""),
                }
                current_grid_cell = (current_task["row"], current_task["col"])
                log(f"Mission task → {cmd_name} ({current_task['row']},{current_task['col']})")
            elif cmd_name == "end_pass":
                log("GCS end_pass command received")
                end_pass = True
            elif cmd_name == "enter_safe_mode":
                log("GCS enter_safe_mode during imaging", level="WARN")
                return images_this_pass, rejections, current_grid_cell, current_task, nadir_locked, True
            elif cmd_name == "adjust_exposure":
                exposure_us = cmd.get("exposure_us")
                if exposure_us:
                    camera._cam.set_controls({"AeEnable": False, "ExposureTime": int(exposure_us)})
                    log(f"Exposure adjusted to {exposure_us} µs by GCS command")
            # Other commands are handled by the main loop after imaging.

        if end_pass:
            break

        # --- Process operator terminal input ---
        op_line = get_operator_input()
        if op_line:
            if op_line.startswith("cell"):
                parts = op_line.split()
                try:
                    current_grid_cell = (int(parts[1]), int(parts[2]))
                    current_task = None
                    log(f"Grid cell → {current_grid_cell} (operator input)")
                except (IndexError, ValueError):
                    log("Bad cell command — format: cell R C", level="WARN")
            elif op_line == "end_pass":
                log("Operator end_pass — stopping imaging")
                end_pass = True
                break

        # --- Hard stops ---
        if len(images_this_pass) >= MAX_IMAGES_PER_PASS:
            log(f"Max images per pass ({MAX_IMAGES_PER_PASS}) reached — ending imaging")
            break

        if storage.is_critical():
            log("Storage critical — stopping imaging early", level="WARN")
            break

        if thermal.is_critical():
            log(f"THERMAL CRITICAL ({thermal.get_cpu_temp():.1f}°C) — entering safe mode", level="ERROR")
            return images_this_pass, rejections, current_grid_cell, current_task, nadir_locked, True

        # --- Adaptive capture interval ---
        interval = CAPTURE_INTERVAL_SEC
        if thermal.is_warning():
            interval *= 2
            log(f"THERMAL WARNING ({thermal.get_cpu_temp():.1f}°C) — capture interval doubled to {interval:.1f}s", level="WARN")

        # --- IMU nadir check with hysteresis ---
        nadir_angle = imu.get_nadir_angle()

        if not nadir_locked:
            if nadir_angle < NADIR_TOLERANCE_DEG:
                nadir_locked = True
                log(f"Nadir locked (angle={nadir_angle:.1f}° < {NADIR_TOLERANCE_DEG}°)")
        else:
            if nadir_angle > NADIR_EXIT_DEG:
                nadir_locked = False
                log(f"Nadir lost (angle={nadir_angle:.1f}° > {NADIR_EXIT_DEG}°)")

        # --- IMU stability check ---
        stable = imu.is_stable()

        if not stable:
            log(f"  Skip: not stable (rate={imu.get_angular_rate():.2f} rad/s)")
            time.sleep(interval)
            continue

        if not nadir_locked:
            log(f"  Skip: not nadir (angle={nadir_angle:.1f}°)")
            time.sleep(interval)
            continue

        # --- CAPTURE ---
        seq = len(images_this_pass)
        ts = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
        fname = f"pass{pass_number}_img{seq:02d}_{ts}.jpg"
        fpath = os.path.join(IMAGE_DIR, fname)

        jpeg_quality = camera.capture_with_recompress(fpath, JPEG_MAX_SIZE_BYTES)

        # Read IMU and camera state immediately after capture for accurate metadata.
        ang_rate = imu.get_angular_rate()
        nadir_angle_post = imu.get_nadir_angle()
        orientation = imu.get_orientation()
        ang_vel = imu.get_angular_velocity()   # [rx, ry, rz] deg/s for GCS mosaic
        cam_meta = camera.get_metadata()
        novelty = coverage.get_novelty(current_grid_cell)

        # --- Quality gate ---
        score, status, details = quality_gate.score_image(fpath, ang_rate, novelty)

        task_match = 1.0 if (
            current_task
            and (current_task.get("row"), current_task.get("col")) == current_grid_cell
        ) else 0.0
        revisit_bonus = 1.0 if current_task and current_task.get("cmd") == "revisit_cell" and task_match else 0.0
        science_value = (
            SCIENCE_WEIGHT_QUALITY * float(score) +
            SCIENCE_WEIGHT_NOVELTY * float(novelty) +
            SCIENCE_WEIGHT_TASK_MATCH * task_match +
            SCIENCE_WEIGHT_REVISIT * revisit_bonus
        )
        science_reasons = []
        if task_match:
            science_reasons.append("matches active GCS task")
        if revisit_bonus:
            science_reasons.append("requested revisit for confirmation")
        if novelty >= 0.8:
            science_reasons.append("high coverage novelty")
        if score >= 0.75:
            science_reasons.append("strong image quality")
        if not science_reasons:
            science_reasons.append("routine survey capture")
        science_block = {
            "science_value": round(science_value, 3),
            "science_reasons": science_reasons,
            "task": current_task,
            "summary": {
                "filename": fname,
                "timestamp": datetime.now(timezone.utc).isoformat(),
                "pass_number": pass_number,
                "grid_cell": list(current_grid_cell),
                "science_value": round(science_value, 3),
                "science_reasons": science_reasons,
                "hazard_hint": "UNKNOWN",
                "change_hint": 0,
                "capture_quality": round(float(score), 3),
                "novelty": round(float(novelty), 3),
            },
        }

        # --- Metadata sidecar ---
        meta = metadata_builder.build(
            filename=fname,
            pass_number=pass_number,
            image_sequence=seq,
            grid_cell=current_grid_cell,
            orientation=orientation,
            angular_rate=ang_rate,
            nadir_locked=nadir_locked,
            nadir_angle_deg=nadir_angle_post,
            cam_meta=cam_meta,
            jpeg_quality=jpeg_quality,
            quality_details=details,
            quality_score=score,
            filepath=fpath,
            angular_velocity=ang_vel,
            science=science_block,
        )
        sidecar_path = fpath.replace(".jpg", "_meta.json")
        metadata_builder.save(meta, sidecar_path)

        # --- Accept or reject ---
        if score > 0:
            # Assign priority tier from novelty score.
            tier = "P1" if novelty >= 0.8 else ("P2" if novelty >= 0.3 else "P3")
            meta["priority_tier"] = tier
            meta["downlink_status"] = "pending"
            storage.update_image_index(fname, meta)
            coverage.update(current_grid_cell, score, fname)
            images_this_pass.append(meta)
            log(
                f"  PASS {fname} | score={score:.2f} tier={tier} "
                f"cell={current_grid_cell} novelty={novelty:.1f}"
            )
        else:
            meta["downlink_status"] = "rejected"
            storage.update_image_index(fname, meta)
            reason = details.get("rejection_reason", "unknown")
            if "blur" in reason:
                rejections["blur"] += 1
            elif "under" in reason:
                rejections["underexposed"] += 1
            elif "over" in reason:
                rejections["overexposed"] += 1
            elif "motion" in reason:
                rejections["motion_blur"] += 1
            log(f"  REJECT {fname} — {status} (reason={reason})")

        time.sleep(interval)

    return images_this_pass, rejections, current_grid_cell, current_task, nadir_locked, False
