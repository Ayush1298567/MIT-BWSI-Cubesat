#!/usr/bin/env python3
"""test_quality.py — Blur threshold calibration tool for MuraltZ CubeSat.

Takes photos of the actual surface and reports Laplacian blur variance,
exposure mean, and quality scores. Use this to calibrate BLUR_THRESHOLD
in config.py before demo day.

CALIBRATION PROCEDURE (from docs/ARCHITECTURE.md section 10):
  Step 1 — Sharp photos:
      Hold the CubeSat steady above the surface. Run this script.
      Record the blur variance values — these are your SHARP baseline.

  Step 2 — Blurry photos:
      Shake the CubeSat gently during capture. Run this script again.
      Record the blur variance values — these are your BLURRY baseline.

  Step 3 — Set threshold:
      BLUR_THRESHOLD = midpoint between lowest sharp and highest blurry variance.
      Example: sharp min=180, blurry max=40 → threshold = (180+40)/2 = 110

  NOTE: Sand and regolith textures often have LOW Laplacian variance even when
  sharp, because the surface has little high-frequency detail. The default of 50
  in config.py is a conservative guess — it WILL need adjustment.

Usage:
    cd /home/cubesat/cubesat_flight
    python3 test_quality.py           # 5 photos (default)
    python3 test_quality.py 10        # 10 photos
    python3 test_quality.py 5 3       # 5 photos, 3-second interval
"""

import os
import sys
import time
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import cv2
import numpy as np

from config import (
    BLUR_THRESHOLD,
    EXPOSURE_MAX,
    EXPOSURE_MIN,
    IMAGE_HEIGHT,
    IMAGE_WIDTH,
    JPEG_QUALITY,
    ANGULAR_RATE_THRESHOLD,
    QUALITY_WEIGHT_BLUR,
    QUALITY_WEIGHT_EXPOSURE,
    QUALITY_WEIGHT_MOTION,
    QUALITY_WEIGHT_NOVELTY,
)


def laplacian_variance(gray):
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def blur_score(variance):
    if variance < BLUR_THRESHOLD:
        return None  # Hard fail
    return min(1.0, (variance - BLUR_THRESHOLD) / (BLUR_THRESHOLD * 3))


def exposure_score(mean):
    if mean < EXPOSURE_MIN or mean > EXPOSURE_MAX:
        return None  # Hard fail
    return max(0.0, 1.0 - abs(mean - 127.5) / 127.5)


def _bar(value, width=30):
    """Simple ASCII progress bar for a 0.0–1.0 value."""
    if value is None:
        return "[" + " " * width + "] N/A"
    filled = int(round(value * width))
    return "[" + "█" * filled + "░" * (width - filled) + f"] {value:.3f}"


def take_photos(n_photos, interval_sec):
    from sensors.camera import Camera

    print(f"\nInitialising camera ({IMAGE_WIDTH}×{IMAGE_HEIGHT}) …")
    cam = Camera(IMAGE_WIDTH, IMAGE_HEIGHT)
    print("Camera ready (2s warm-up included)\n")

    results = []
    tmp_dir = tempfile.gettempdir()

    for i in range(n_photos):
        fname = os.path.join(tmp_dir, f"qtest_{i:02d}.jpg")
        print(f"Photo {i + 1}/{n_photos} … ", end="", flush=True)
        cam.capture(fname)
        meta = cam.get_metadata()
        size = os.path.getsize(fname)
        print(f"saved ({size // 1024} KB)  ExposureTime={meta.get('ExposureTime')} µs  Gain={meta.get('AnalogueGain'):.2f}")

        results.append({
            "index": i + 1,
            "filepath": fname,
            "file_size": size,
            "cam_meta": meta,
        })

        if i < n_photos - 1:
            time.sleep(interval_sec)

    cam.close()
    return results


def analyse_photos(results, imu=None):
    print("\n" + "=" * 72)
    print("  QUALITY ANALYSIS")
    print("=" * 72)

    blur_variances = []
    exposure_means = []

    for r in results:
        idx = r["index"]
        path = r["filepath"]

        img = cv2.imread(path)
        if img is None:
            print(f"\nPhoto {idx}: could not decode — skipping")
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blur_var = laplacian_variance(gray)
        exp_mean = float(np.mean(gray))

        b_score = blur_score(blur_var)
        e_score = exposure_score(exp_mean)

        # Motion score: 0.0 if no IMU, or from live IMU reading.
        if imu is not None:
            ang_rate = imu.get_angular_rate()
            if ang_rate >= ANGULAR_RATE_THRESHOLD:
                m_score = None
            else:
                m_score = 1.0 - (ang_rate / ANGULAR_RATE_THRESHOLD)
        else:
            ang_rate = 0.0
            m_score = 1.0  # Assumed stable if no IMU

        novelty = 1.0  # Assume new cell for calibration purposes

        # Combined score (only if no hard fails).
        if b_score is not None and e_score is not None and m_score is not None:
            combined = (
                QUALITY_WEIGHT_BLUR * b_score
                + QUALITY_WEIGHT_EXPOSURE * e_score
                + QUALITY_WEIGHT_MOTION * m_score
                + QUALITY_WEIGHT_NOVELTY * novelty
            )
        else:
            combined = None

        blur_variances.append(blur_var)
        exposure_means.append(exp_mean)

        cam_meta = r["cam_meta"]
        print(f"\n  ┌── Photo {idx} ──────────────────────────────────────────────┐")
        print(f"  │  File size     : {r['file_size']:,} bytes ({r['file_size'] // 1024} KB)")
        print(f"  │  ExposureTime  : {cam_meta.get('ExposureTime')} µs")
        print(f"  │  AnalogueGain  : {cam_meta.get('AnalogueGain')}")
        print(f"  │  Lux           : {cam_meta.get('Lux')}")
        print(f"  │")
        print(f"  │  Blur variance : {blur_var:>8.2f}  (BLUR_THRESHOLD={BLUR_THRESHOLD})")

        if b_score is None:
            print(f"  │  Blur score    :   HARD FAIL  (variance < threshold — image too blurry)")
        else:
            print(f"  │  Blur score    : {_bar(b_score)}")

        print(f"  │  Exposure mean : {exp_mean:>8.2f}  (valid range {EXPOSURE_MIN}–{EXPOSURE_MAX})")

        if e_score is None:
            verdict = "underexposed" if exp_mean < EXPOSURE_MIN else "overexposed"
            print(f"  │  Exposure score:   HARD FAIL  ({verdict})")
        else:
            print(f"  │  Exposure score: {_bar(e_score)}")

        if imu is not None:
            print(f"  │  Angular rate  : {ang_rate:.4f} rad/s")
            if m_score is None:
                print(f"  │  Motion score  :   HARD FAIL  (rate ≥ {ANGULAR_RATE_THRESHOLD} rad/s)")
            else:
                print(f"  │  Motion score  : {_bar(m_score)}")

        print(f"  │")
        if combined is not None:
            print(f"  │  Combined score: {_bar(combined)}")
            tier = "P1" if combined >= 0.8 else ("P2" if combined >= 0.5 else "P3")
            print(f"  │  Priority tier : {tier}  (novelty assumed 1.0 for calibration)")
        else:
            print(f"  │  Combined score: REJECTED (hard fail on one or more checks)")
        print(f"  └────────────────────────────────────────────────────────────┘")

        # Clean up temp file.
        try:
            os.remove(path)
        except OSError:
            pass

    return blur_variances, exposure_means


def print_calibration_guide(blur_variances):
    print("\n" + "=" * 72)
    print("  BLUR THRESHOLD CALIBRATION GUIDE")
    print("=" * 72)

    if not blur_variances:
        print("  No blur data collected.")
        return

    min_var = min(blur_variances)
    max_var = max(blur_variances)
    avg_var = sum(blur_variances) / len(blur_variances)

    print(f"\n  From this run ({len(blur_variances)} photos):")
    print(f"    Blur variance min : {min_var:.2f}")
    print(f"    Blur variance max : {max_var:.2f}")
    print(f"    Blur variance avg : {avg_var:.2f}")
    print(f"    Current threshold : {BLUR_THRESHOLD}")

    print(f"""
  CALIBRATION STEPS:
  ──────────────────
  1. Run this script with CubeSat HELD STILL above the surface.
     Record the blur variances — this is your SHARP baseline.
     Note the MINIMUM value: ___________

  2. Run this script while SHAKING the CubeSat during each capture.
     Record the blur variances — this is your BLURRY baseline.
     Note the MAXIMUM value: ___________

  3. Set BLUR_THRESHOLD in config.py to the midpoint:
       BLUR_THRESHOLD = (sharp_min + blurry_max) / 2

  4. Re-run this script held still and confirm all photos PASS.
     Re-run while shaking and confirm all photos FAIL.

  EXAMPLE:
    Sharp photos gave variance 150, 180, 165 → min = 150
    Blurry photos gave variance 20, 35, 28   → max = 35
    Midpoint = (150 + 35) / 2 = 92.5
    Set BLUR_THRESHOLD = 92.5  (or round to 90)

  Your surface has LOW texture (sand/regolith)?
    Expect lower sharp variances than smooth objects.
    A threshold of 30–80 is typical for granular surfaces.
    A threshold of 80–200 is typical for textured objects.
""")


def main():
    n_photos = int(sys.argv[1]) if len(sys.argv) > 1 else 5
    interval_sec = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0

    print("=" * 72)
    print("  MuraltZ CubeSat — Quality Gate Calibration")
    print("=" * 72)
    print(f"\n  Photos     : {n_photos}")
    print(f"  Interval   : {interval_sec}s between captures")
    print(f"  Current BLUR_THRESHOLD in config.py: {BLUR_THRESHOLD}")
    print(f"\n  Hold the CubeSat STILL and pointing DOWN over the surface.")
    print(f"  Press Enter when ready …", end="")
    try:
        input()
    except (KeyboardInterrupt, EOFError):
        print("\nAborted.")
        sys.exit(0)

    # Optional IMU — don't crash if not available.
    imu = None
    try:
        from sensors.imu import IMU
        imu = IMU()
        print("  IMU available — motion scores will be real.")
    except Exception as exc:
        print(f"  IMU unavailable ({exc}) — motion score will be assumed 1.0.")

    try:
        results = take_photos(n_photos, interval_sec)
    except KeyboardInterrupt:
        print("\nAborted during capture.")
        sys.exit(0)

    blur_variances, exposure_means = analyse_photos(results, imu)
    print_calibration_guide(blur_variances)


if __name__ == "__main__":
    main()
