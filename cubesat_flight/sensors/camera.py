# sensors/camera.py — Pi Camera Module 3 wrapper via picamera2
#
# Hardware: Pi Camera Module 3 (IMX708), connected via CSI ribbon.
# Import: from picamera2 import Picamera2. NOT libcamera directly.
# Camera starts in AUTO-EXPOSURE mode and stays there by default.
# Manual exposure is only set via set_low_light_mode() or adjust_exposure command.

import os
import time
import cv2
from picamera2 import Picamera2

from config import JPEG_QUALITY
from utils.logger import log


class Camera:
    def __init__(self, width, height):
        """Initialise camera in auto-exposure still-capture mode.
        Blocks for 2 seconds while the sensor warms up and AGC settles."""
        self._width = width
        self._height = height
        self._last_metadata = {}

        self._cam = Picamera2()
        config = self._cam.create_still_configuration(
            main={"size": (width, height), "format": "RGB888"}
        )
        self._cam.configure(config)
        # Auto-exposure is the default — no controls set here.
        self._cam.start()
        time.sleep(2)  # Allow AGC and AWB to settle before first capture

    def capture(self, filepath):
        """Capture a JPEG at JPEG_QUALITY and save to filepath.
        Stores the real camera metadata (exposure, gain, lux) for get_metadata()."""
        self._last_metadata = self._cam.capture_file(
            filepath,
            options={"quality": JPEG_QUALITY}
        )
        return filepath

    def capture_with_recompress(self, filepath, max_bytes):
        """Capture at JPEG_QUALITY. If the file exceeds max_bytes, re-encode
        the saved JPEG at (JPEG_QUALITY - 10) in place using OpenCV and log it.
        Returns the final JPEG quality used."""
        self._last_metadata = self._cam.capture_file(
            filepath,
            options={"quality": JPEG_QUALITY}
        )

        file_size = os.path.getsize(filepath)
        if file_size <= max_bytes:
            return JPEG_QUALITY

        # Re-encode the on-disk JPEG at lower quality — no second capture needed.
        fallback_quality = JPEG_QUALITY - 10
        img = cv2.imread(filepath)
        cv2.imwrite(filepath, img, [cv2.IMWRITE_JPEG_QUALITY, fallback_quality])
        new_size = os.path.getsize(filepath)
        log(
            f"Recompressed {os.path.basename(filepath)} "
            f"Q{JPEG_QUALITY}→Q{fallback_quality} "
            f"({file_size // 1024}KB→{new_size // 1024}KB)",
            level="INFO"
        )
        return fallback_quality

    def set_low_light_mode(self):
        """Switch to fixed manual exposure for eclipse / low-light demo segment.
        ExposureTime 100 ms (100,000 µs), AnalogueGain 8.0."""
        self._cam.set_controls({
            "AeEnable": False,
            "ExposureTime": 100_000,    # µs
            "AnalogueGain": 8.0,
        })

    def set_normal_mode(self):
        """Return to auto-exposure (AEC/AGC). Reverts any manual exposure set."""
        self._cam.set_controls({
            "AeEnable": True,
        })

    def get_metadata(self):
        """Return the metadata dict from the most recent capture.
        Includes real sensor values: ExposureTime (µs), AnalogueGain, Lux, etc.
        Returns an empty dict if no capture has been taken yet."""
        return self._last_metadata

    def close(self):
        """Stop the camera cleanly. Call this before process exit."""
        self._cam.stop()
        self._cam.close()
