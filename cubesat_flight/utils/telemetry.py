# utils/telemetry.py — Telemetry packet builder
#
# build_telemetry() assembles the full telemetry dict from live objects.
# Every field comes from a real hardware source or live software state.
# The returned dict is sent over TCP as the first packet of each downlink window
# and also on demand in response to a "status_request" ground command.

import time
from datetime import datetime, timezone

from config import CUBESAT_ID, THROTTLE_BYTES_PER_SEC, DOWNLINK_WINDOW_SEC
from utils.logger import get_recent

_boot_time = time.monotonic()


def build_telemetry(
    imu,
    camera,
    thermal,
    storage,
    coverage,
    queue,
    pass_number,
    state,
    rejections,
    captured_this_pass,
    captured_total,
    rejected_total,
    bytes_sent_this_pass,
    images_sent_total,
    gcs_reachable,
    nadir_locked,
    errors=None,
):
    """Build the full telemetry packet dict from live objects and state.

    Args:
        imu:                IMU instance
        camera:             Camera instance
        thermal:            Thermal instance
        storage:            StorageManager instance
        coverage:           CoverageTracker instance
        queue:              Current downlink queue (list of metadata dicts)
        pass_number:        Current pass number (int)
        state:              Current state machine state as string ("IMAGING", etc.)
        rejections:         Dict of rejection counts {"blur": N, "underexposed": N, ...}
        captured_this_pass: Images captured so far this pass (int)
        captured_total:     Total images captured across all passes (int)
        rejected_total:     Total images rejected across all passes (int)
        bytes_sent_this_pass: Bytes sent in this downlink window (int)
        images_sent_total:  Total images successfully sent (int)
        gcs_reachable:      Whether GCS is currently reachable (bool)
        nadir_locked:       Whether nadir lock is currently active (bool)
        errors:             Optional list of recent error strings

    Returns:
        dict — JSON-serialisable telemetry packet matching the architecture schema.
    """
    # --- IMU ---
    accel = imu.get_acceleration()
    gyro = imu.get_gyro()
    angular_rate = imu.get_angular_rate()
    nadir_angle = imu.get_nadir_angle()

    # --- Camera ---
    cam_meta = camera.get_metadata()

    # --- Thermal ---
    cpu_temp = thermal.get_cpu_temp()

    # --- Storage ---
    capacity = storage.check_capacity()

    # --- Coverage ---
    cov_summary = coverage.get_summary()

    # --- Data budget ---
    data_budget = THROTTLE_BYTES_PER_SEC * DOWNLINK_WINDOW_SEC  # 72,000 bytes
    budget_remaining = max(0, data_budget - bytes_sent_this_pass)

    # --- Uptime ---
    uptime_sec = int(time.monotonic() - _boot_time)

    return {
        "type": "telemetry",
        "timestamp": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "cubesat_id": CUBESAT_ID,
        "pass_number": pass_number,
        "state": state,
        "uptime_sec": uptime_sec,

        "imu": {
            "accel": [round(v, 4) for v in accel],
            "gyro": [round(v, 4) for v in gyro],
            "angular_rate": round(angular_rate, 4),
            "stable": imu.is_stable(),
            "nadir_locked": nadir_locked,
            "nadir_angle_deg": round(nadir_angle, 2),
        },

        "camera": {
            "exposure_us": cam_meta.get("ExposureTime"),
            "analog_gain": cam_meta.get("AnalogueGain"),
            "lux": cam_meta.get("Lux"),
            "mode": "auto" if cam_meta.get("AeEnable", True) else "manual",
        },

        "thermal": {
            "cpu_temp_c": round(cpu_temp, 1),
            "throttled": thermal.is_warning(),
        },

        "storage": {
            "used_pct": capacity["used_pct"],
            "free_mb": capacity["free_mb"],
        },

        "imaging": {
            "captured_this_pass": captured_this_pass,
            "captured_total": captured_total,
            "rejected_total": rejected_total,
            "rejection_breakdown": rejections,
        },

        "downlink": {
            "queued": len(queue),
            "sent_total": images_sent_total,
            "bytes_this_pass": bytes_sent_this_pass,
            "budget_remaining": budget_remaining,
            "gcs_reachable": gcs_reachable,
        },

        "coverage": {
            "cells_filled": cov_summary["cells_filled"],
            "cells_total": cov_summary["cells_total"],
            "pct": cov_summary["pct"],
        },

        "errors": errors or [],
        "recent_log": get_recent(),
    }
