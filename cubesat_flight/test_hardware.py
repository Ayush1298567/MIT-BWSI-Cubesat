#!/usr/bin/env python3
"""test_hardware.py — Hardware verification script for MuraltZ CubeSat.

Run this on the Raspberry Pi before starting main.py to confirm every
hardware component is functional and configured correctly.

Usage:
    cd /home/cubesat/cubesat_flight
    python3 test_hardware.py
"""

import os
import shutil
import socket
import sys
import tempfile
import time

# Allow running from the cubesat_flight/ directory directly.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from config import (
    CAMERA_FOV_H_DEG,
    CAMERA_FOV_V_DEG,
    CPU_TEMP_CRITICAL_C,
    CPU_TEMP_WARNING_C,
    DATA_PORT,
    GROUND_STATION_IP,
    IMAGE_HEIGHT,
    IMAGE_WIDTH,
    JPEG_QUALITY,
    STORAGE_CRITICAL_PCT,
    STORAGE_WARNING_PCT,
)

_PASS = "  [PASS]"
_FAIL = "  [FAIL]"
_WARN = "  [WARN]"

_results = {}


def _record(name, passed, detail=""):
    _results[name] = passed
    tag = _PASS if passed else _FAIL
    line = f"{tag}  {name}"
    if detail:
        line += f" — {detail}"
    print(line)


def section(title):
    print(f"\n{'─' * 56}")
    print(f"  {title}")
    print(f"{'─' * 56}")


# ---------------------------------------------------------------------------
# 1. IMU
# ---------------------------------------------------------------------------

def test_imu():
    section("1. IMU — LSM6DSO32 @ I2C 0x6A")
    try:
        from sensors.imu import IMU
        imu = IMU()

        ax, ay, az = imu.get_acceleration()
        gx, gy, gz = imu.get_gyro()
        rate = imu.get_angular_rate()
        nadir = imu.get_nadir_angle()
        orientation = imu.get_orientation()

        print(f"  Acceleration  : x={ax:+.3f}  y={ay:+.3f}  z={az:+.3f}  m/s²")
        print(f"  Gyro          : x={gx:+.4f}  y={gy:+.4f}  z={gz:+.4f}  rad/s")
        print(f"  Angular rate  : {rate:.4f} rad/s")
        print(f"  Nadir angle   : {nadir:.1f}°  (0° = pointing straight down)")
        print(f"  Roll          : {orientation['roll']:.2f}°")
        print(f"  Pitch         : {orientation['pitch']:.2f}°")
        print(f"  Yaw           : {orientation['yaw']}  (no magnetometer — expected None)")
        print(f"  Accel mag     : {orientation['accel_mag']:.4f} m/s²")

        accel_ok = 8.0 < abs(az) < 11.0
        if not accel_ok:
            _record("IMU accel Z (gravity check)", False,
                    f"|az|={abs(az):.2f} m/s² — expected 8.0–11.0. Is CubeSat upright?")
        else:
            _record("IMU accel Z (gravity check)", True,
                    f"|az|={abs(az):.2f} m/s² ✓")

        _record("IMU stable at rest", rate < 1.0,
                f"angular_rate={rate:.4f} rad/s (need < 1.0)")

        _record("IMU nadir angle", nadir < 45.0,
                f"{nadir:.1f}° (need < 45° for capture gate)")

        return imu

    except Exception as exc:
        _record("IMU init", False, str(exc))
        print(f"  Exception: {exc}")
        return None


# ---------------------------------------------------------------------------
# 2. Camera
# ---------------------------------------------------------------------------

def test_camera():
    section("2. Camera — Pi Camera Module 3 (picamera2)")
    try:
        from sensors.camera import Camera
        import cv2

        print(f"  Initialising camera ({IMAGE_WIDTH}×{IMAGE_HEIGHT}) …")
        cam = Camera(IMAGE_WIDTH, IMAGE_HEIGHT)
        print("  Camera started (2-second warm-up included)")

        test_path = os.path.join(tempfile.gettempdir(), "cubesat_hw_test.jpg")
        cam.capture(test_path)

        size = os.path.getsize(test_path)
        meta = cam.get_metadata()

        print(f"  Test image    : {test_path}")
        print(f"  File size     : {size:,} bytes ({size / 1024:.1f} KB)")
        print(f"  ExposureTime  : {meta.get('ExposureTime')} µs")
        print(f"  AnalogueGain  : {meta.get('AnalogueGain')}")
        print(f"  Lux           : {meta.get('Lux')}")
        print(f"  FOV (config)  : {CAMERA_FOV_H_DEG}° H × {CAMERA_FOV_V_DEG}° V")

        img = cv2.imread(test_path)
        decodable = img is not None
        if decodable:
            print(f"  Image shape   : {img.shape}  (H×W×C)")

        _record("Camera captures image", size > 0, f"{size:,} bytes")
        _record("Camera image > 10 KB", size > 10_000, f"{size / 1024:.1f} KB")
        _record("Camera image decodable by OpenCV", decodable)
        _record("Camera metadata present", bool(meta.get("ExposureTime")),
                f"ExposureTime={meta.get('ExposureTime')} µs")

        try:
            os.remove(test_path)
        except OSError:
            pass

        cam.close()
        return True

    except Exception as exc:
        _record("Camera init/capture", False, str(exc))
        print(f"  Exception: {exc}")
        return False


# ---------------------------------------------------------------------------
# 3. CPU Temperature
# ---------------------------------------------------------------------------

def test_thermal():
    section("3. CPU Temperature — /sys/class/thermal/thermal_zone0/temp")
    try:
        from utils.thermal import Thermal

        thermal = Thermal()
        thermal.start_monitoring()
        time.sleep(0.5)  # Let thread do one read

        temp = thermal.get_cpu_temp()
        print(f"  CPU temp      : {temp:.1f}°C")
        print(f"  Warning level : {CPU_TEMP_WARNING_C}°C")
        print(f"  Critical level: {CPU_TEMP_CRITICAL_C}°C")
        print(f"  Is warning    : {thermal.is_warning()}")
        print(f"  Is critical   : {thermal.is_critical()}")

        readable = temp > 0
        _record("CPU temp readable", readable, f"{temp:.1f}°C")

        if thermal.is_critical():
            _record("CPU temp within safe range", False,
                    f"{temp:.1f}°C ≥ {CPU_TEMP_CRITICAL_C}°C CRITICAL")
        elif thermal.is_warning():
            print(f"{_WARN}  CPU temp elevated ({temp:.1f}°C) — consider cooling before long run")
            _record("CPU temp within safe range", True, f"{temp:.1f}°C (warning, not critical)")
        else:
            _record("CPU temp within safe range", True, f"{temp:.1f}°C ✓")

        return thermal

    except Exception as exc:
        _record("CPU temp read", False, str(exc))
        print(f"  Exception: {exc}")
        return None


# ---------------------------------------------------------------------------
# 4. Ground Station Ping
# ---------------------------------------------------------------------------

def test_gcs():
    section(f"4. Ground Station — {GROUND_STATION_IP}:{DATA_PORT}")
    try:
        print(f"  Attempting TCP connect to {GROUND_STATION_IP}:{DATA_PORT} …")
        start = time.monotonic()
        with socket.create_connection((GROUND_STATION_IP, DATA_PORT), timeout=3):
            rtt = (time.monotonic() - start) * 1000
            _record("GCS reachable", True, f"connected in {rtt:.0f} ms")
            return True
    except OSError as exc:
        _record("GCS reachable", False,
                f"connection refused/timeout — {exc}. "
                "Start GCS server, or this is expected in autonomous mode.")
        print(f"  NOTE: GCS unreachable is non-fatal. main.py will run in autonomous mode.")
        return False


# ---------------------------------------------------------------------------
# 5. Storage / Disk Capacity
# ---------------------------------------------------------------------------

def test_storage():
    section("5. Storage — Disk Capacity")
    try:
        from storage.manager import StorageManager
        from config import IMAGE_DIR

        os.makedirs(IMAGE_DIR, exist_ok=True)
        storage = StorageManager()
        cap = storage.check_capacity()

        print(f"  Partition     : {IMAGE_DIR}")
        print(f"  Used          : {cap['used_pct']:.1f}%")
        print(f"  Free          : {cap['free_mb']:.0f} MB")
        print(f"  Total         : {cap['total_mb']:.0f} MB")
        print(f"  Warning at    : {STORAGE_WARNING_PCT}%")
        print(f"  Critical at   : {STORAGE_CRITICAL_PCT}%")

        _record("Storage readable", True, f"{cap['used_pct']:.1f}% used")

        if cap["used_pct"] > STORAGE_CRITICAL_PCT:
            _record("Storage below critical", False,
                    f"{cap['used_pct']:.1f}% ≥ {STORAGE_CRITICAL_PCT}% — free space immediately")
        elif cap["used_pct"] > STORAGE_WARNING_PCT:
            print(f"{_WARN}  Storage above warning ({cap['used_pct']:.1f}%) — cleanup P3 images soon")
            _record("Storage below warning", False,
                    f"{cap['used_pct']:.1f}% — imaging will still work")
        else:
            _record("Storage below warning", True, f"{cap['free_mb']:.0f} MB free ✓")

    except Exception as exc:
        _record("Storage check", False, str(exc))
        print(f"  Exception: {exc}")


# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------

def print_summary():
    section("SUMMARY")
    passed = sum(1 for v in _results.values() if v)
    total = len(_results)
    failed = [name for name, ok in _results.items() if not ok]

    for name, ok in _results.items():
        tag = _PASS if ok else _FAIL
        print(f"{tag}  {name}")

    print(f"\n  {passed}/{total} checks passed")

    if failed:
        print(f"\n  FAILED checks:")
        for name in failed:
            print(f"    • {name}")
        print()
        # GCS failure alone is not a blocker.
        blockers = [n for n in failed if "GCS" not in n]
        if blockers:
            print("  ✗ Hardware issues detected. Fix before running main.py.")
            return False
        else:
            print("  ✓ Only GCS unreachable — autonomous mode will work.")
            return True
    else:
        print("\n  ✓ All checks passed. Ready to run main.py.")
        return True


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    print("=" * 56)
    print("  MuraltZ CubeSat — Hardware Verification")
    print("=" * 56)

    test_imu()
    test_camera()
    test_thermal()
    test_gcs()
    test_storage()

    ok = print_summary()
    sys.exit(0 if ok else 1)
