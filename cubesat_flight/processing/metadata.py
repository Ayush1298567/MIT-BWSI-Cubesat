# processing/metadata.py — _meta.json sidecar generator
#
# MetadataBuilder assembles the full metadata dict for each captured image
# and writes it to disk as a JSON sidecar file alongside the JPEG.
#
# Schema follows docs/ARCHITECTURE.md section 7.5.
# Yaw is intentionally omitted — the LSM6DSO32 has no magnetometer.

import hashlib
import json
import os
from datetime import datetime, timezone


class MetadataBuilder:
    def build(
        self,
        filename,
        pass_number,
        image_sequence,
        grid_cell,
        orientation,
        angular_rate,
        nadir_locked,
        nadir_angle_deg,
        cam_meta,
        jpeg_quality,
        quality_details,
        quality_score,
        filepath,
    ):
        """Assemble the complete metadata dict for one captured image.

        Args:
            filename:        Bare filename, e.g. "pass3_img07_20260315_144500.jpg"
            pass_number:     Integer pass counter (1-indexed)
            image_sequence:  Index of this image within the pass (0-indexed)
            grid_cell:       (row, col) tuple — from operator-set current_grid_cell
            orientation:     Dict from imu.get_orientation() — roll, pitch, accel_mag, angular_rate
            angular_rate:    Float rad/s from imu.get_angular_rate() at capture time
            nadir_locked:    Bool — was nadir lock active at capture time?
            nadir_angle_deg: Float degrees — raw nadir angle at capture time
            cam_meta:        Dict from camera.get_metadata() — real picamera2 metadata
            jpeg_quality:    Int — actual JPEG quality used (70 or 60 if recompressed)
            quality_details: Dict from QualityGate.score_image()
            quality_score:   Float combined quality score (0.0–1.0)
            filepath:        Full path to the JPEG — used to compute file_size and md5

        Returns:
            dict matching the _meta.json schema. Callers should store this dict
            and later call save() or update it with priority_tier / downlink_status.
        """
        file_size = os.path.getsize(filepath) if os.path.exists(filepath) else 0
        md5 = self._md5(filepath) if os.path.exists(filepath) else ""
        timestamp = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")

        # IMU block — yaw is excluded: no magnetometer on this hardware.
        imu_block = {
            "roll": orientation.get("roll"),
            "pitch": orientation.get("pitch"),
            # yaw intentionally omitted — LSM6DSO32 has no magnetometer
            "accel_magnitude": orientation.get("accel_mag"),
            "angular_rate": round(angular_rate, 4),
            "stable": angular_rate < 1.0,   # mirrors is_stable() logic
            "nadir_locked": nadir_locked,
            "nadir_angle_deg": round(nadir_angle_deg, 2),
        }

        # Camera block — pulled from real picamera2 metadata.
        # picamera2 key names: ExposureTime (µs), AnalogueGain, Lux
        cam_block = {
            "exposure_us": cam_meta.get("ExposureTime"),
            "analog_gain": cam_meta.get("AnalogueGain"),
            "lux": cam_meta.get("Lux"),
            "jpeg_quality": jpeg_quality,
            "recompressed": jpeg_quality < 70,
        }

        # Quality block — sourced directly from QualityGate.score_image() details.
        quality_block = {
            "blur_variance": quality_details.get("blur_variance"),
            "blur_score": quality_details.get("blur_score"),
            "exposure_mean": quality_details.get("exposure_mean"),
            "exposure_score": quality_details.get("exposure_score"),
            "motion_score": quality_details.get("motion_score"),
            "novelty_score": quality_details.get("novelty_score"),
            "combined_score": quality_score,
            "passed_gate": quality_details.get("passed_gate", False),
            "rejection_reason": quality_details.get("rejection_reason"),
        }

        return {
            "filename": filename,
            "pass_number": pass_number,
            "image_sequence": image_sequence,
            "timestamp": timestamp,
            "grid_cell": list(grid_cell),   # [row, col] — JSON-serialisable
            "imu": imu_block,
            "camera": cam_block,
            "quality": quality_block,
            "priority_tier": None,          # Set by caller after novelty scoring
            "file_size_bytes": file_size,
            "md5": md5,
            "downlink_status": "pending",
        }

    def save(self, meta_dict, filepath):
        """Write the metadata dict to a JSON sidecar file.

        Args:
            meta_dict: Dict from build().
            filepath:  Full path for the sidecar, e.g.:
                       "/home/.../data/images/pass3_img07_20260315_144500_meta.json"
        """
        os.makedirs(os.path.dirname(filepath), exist_ok=True)
        with open(filepath, "w") as f:
            json.dump(meta_dict, f, indent=2)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _md5(filepath):
        """Compute MD5 hex digest of a file for transfer integrity checking."""
        h = hashlib.md5()
        with open(filepath, "rb") as f:
            for chunk in iter(lambda: f.read(65536), b""):
                h.update(chunk)
        return h.hexdigest()
