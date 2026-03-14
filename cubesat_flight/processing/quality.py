# processing/quality.py — Image quality gate
#
# QualityGate scores each captured image on four axes:
#   blur       — Laplacian variance (higher = sharper)
#   exposure   — mean pixel brightness (0–255 range, ideal ~128)
#   motion     — angular rate from IMU at capture time (lower = less motion blur)
#   novelty    — passed in from CoverageTracker (new cell vs redundant)
#
# Combined: Q = 0.3*blur + 0.25*exposure + 0.2*motion + 0.25*novelty
# Any hard fail → score = 0.0 and image is rejected.

import cv2
import numpy as np

from config import (
    BLUR_THRESHOLD,
    EXPOSURE_MIN,
    EXPOSURE_MAX,
    ANGULAR_RATE_THRESHOLD,
    QUALITY_WEIGHT_BLUR,
    QUALITY_WEIGHT_EXPOSURE,
    QUALITY_WEIGHT_MOTION,
    QUALITY_WEIGHT_NOVELTY,
)


class QualityGate:
    def score_image(self, filepath, angular_rate, novelty_score):
        """Score a captured JPEG on four quality axes.

        Args:
            filepath:      Path to the JPEG on disk.
            angular_rate:  IMU angular rate magnitude (rad/s) at capture time.
            novelty_score: Float from CoverageTracker — 1.0 new, 0.5 better, 0.1 redundant.

        Returns:
            (score, status, details)
            score   float  0.0–1.0 combined quality score. 0.0 means rejected.
            status  str    "passed" or a short rejection label.
            details dict   Individual scores, raw sensor values, rejection_reason.
        """
        details = {
            "blur_variance": None,
            "blur_score": 0.0,
            "exposure_mean": None,
            "exposure_score": 0.0,
            "motion_score": 0.0,
            "novelty_score": novelty_score,
            "combined_score": 0.0,
            "passed_gate": False,
            "rejection_reason": None,
        }

        # --- Load image ---
        img = cv2.imread(filepath)
        if img is None:
            details["rejection_reason"] = "unreadable_file"
            return 0.0, "rejected_unreadable", details

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # --- Check 1: Blur ---
        blur_variance = float(cv2.Laplacian(gray, cv2.CV_64F).var())
        details["blur_variance"] = round(blur_variance, 2)

        if blur_variance < BLUR_THRESHOLD:
            details["rejection_reason"] = "blur"
            return 0.0, "rejected_blur", details

        # Normalise blur score: reaches 1.0 at 4× the threshold.
        # Images between threshold and 4× threshold score 0.0–1.0 linearly.
        blur_score = min(1.0, (blur_variance - BLUR_THRESHOLD) / (BLUR_THRESHOLD * 3))
        details["blur_score"] = round(blur_score, 4)

        # --- Check 2: Exposure ---
        exposure_mean = float(np.mean(gray))
        details["exposure_mean"] = round(exposure_mean, 2)

        if exposure_mean < EXPOSURE_MIN:
            details["rejection_reason"] = "underexposed"
            return 0.0, "rejected_underexposed", details

        if exposure_mean > EXPOSURE_MAX:
            details["rejection_reason"] = "overexposed"
            return 0.0, "rejected_overexposed", details

        # Score peaks at 128 (mid-grey ideal), falls linearly to 0 at the limits.
        exposure_score = 1.0 - abs(exposure_mean - 127.5) / 127.5
        exposure_score = max(0.0, min(1.0, exposure_score))
        details["exposure_score"] = round(exposure_score, 4)

        # --- Check 3: Motion blur ---
        if angular_rate >= ANGULAR_RATE_THRESHOLD:
            details["rejection_reason"] = "motion_blur"
            return 0.0, "rejected_motion_blur", details

        # Score: 1.0 at rest, falls linearly to 0 as rate approaches threshold.
        motion_score = 1.0 - (angular_rate / ANGULAR_RATE_THRESHOLD)
        motion_score = max(0.0, min(1.0, motion_score))
        details["motion_score"] = round(motion_score, 4)

        # --- Check 4: Novelty (no hard fail — low novelty just lowers score) ---
        # novelty_score is 1.0 (new cell), 0.5 (better quality available), or
        # 0.1 (redundant). Already stored in details above.

        # --- Combined score ---
        combined = (
            QUALITY_WEIGHT_BLUR * blur_score
            + QUALITY_WEIGHT_EXPOSURE * exposure_score
            + QUALITY_WEIGHT_MOTION * motion_score
            + QUALITY_WEIGHT_NOVELTY * novelty_score
        )
        combined = round(max(0.0, min(1.0, combined)), 4)
        details["combined_score"] = combined
        details["passed_gate"] = True

        return combined, "passed", details
