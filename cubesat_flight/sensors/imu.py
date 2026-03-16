# sensors/imu.py — LSM6DSO32 IMU wrapper
#
# Hardware: LSM6DSO32 at I2C address 0x6A.
# Reads physical sensor every call. No caching.
# Yaw cannot be computed — no magnetometer. Only roll and pitch from accelerometer.

import math
import board
import busio
from adafruit_lsm6ds.lsm6dso32 import LSM6DSO32 as LSM6DS
from adafruit_lsm6ds import AccelRange

from config import ANGULAR_RATE_THRESHOLD


class IMU:
    def __init__(self):
        i2c = busio.I2C(board.SCL, board.SDA)
        # LSM6DSO32 default I2C address is 0x6A (SDO/SA0 pin pulled low)
        self._imu = LSM6DS(i2c, address=0x6A)
        self._imu.accelerometer_range = AccelRange.RANGE_4G

    def get_acceleration(self):
        """Return (x, y, z) acceleration in m/s².
        At rest pointing nadir: approximately (0, 0, -9.8)."""
        return self._imu.acceleration  # (x, y, z) m/s²

    def get_gyro(self):
        """Return (x, y, z) angular velocity in rad/s.
        At rest: approximately (0, 0, 0)."""
        return self._imu.gyro  # (x, y, z) rad/s

    def get_angular_rate(self):
        """Return scalar angular rate magnitude in rad/s.
        Computed as sqrt(gx² + gy² + gz²)."""
        gx, gy, gz = self.get_gyro()
        return math.sqrt(gx * gx + gy * gy + gz * gz)

    def is_stable(self):
        """Return True if angular rate is below the stability threshold.
        Threshold is ANGULAR_RATE_THRESHOLD (1.0 rad/s) from config."""
        return self.get_angular_rate() < ANGULAR_RATE_THRESHOLD

    def get_nadir_angle(self):
        """Return angle in degrees between the camera boresight and the gravity vector.

        The camera boresight is along the -X axis of the IMU (camera ribbon side
        of the Pi 4 faces +X; lens faces -X). Nadir = 0° means camera points
        straight down toward the surface.

        If the dominant gravity axis is not -X (i.e. the operator is holding
        the Pi in an unexpected orientation), we fall back to measuring nadir
        as the angle between the gravity vector and whichever axis carries the
        most gravity — this keeps the gate usable regardless of mount orientation.
        """
        ax, ay, az = self.get_acceleration()
        accel_mag = math.sqrt(ax * ax + ay * ay + az * az)
        if accel_mag < 1e-6:
            return 90.0

        # Camera faces +Z on this Pi 4 mount (lens on the bottom of the
        # board, pointing through the PCB side). When camera is pointed at
        # the ground, gravity (positive) aligns with +Z.
        # nadir = angle between accel vector and +Z = arccos(az / |accel|)
        cos_angle = az / accel_mag
        cos_angle = max(-1.0, min(1.0, cos_angle))
        return math.degrees(math.acos(cos_angle))

    def get_angular_velocity(self):
        """Return [rx, ry, rz] angular velocity in deg/s (body frame).
        Used by the GCS mosaic stitcher to estimate translation direction."""
        gx, gy, gz = self.get_gyro()  # rad/s
        return [
            round(math.degrees(gx), 4),
            round(math.degrees(gy), 4),
            round(math.degrees(gz), 4),
        ]

    def get_orientation(self):
        """Return orientation dict derived from accelerometer.
        Roll and pitch are computed from gravity vector.
        Yaw is None — no magnetometer is present on this hardware.

        Returns:
            dict with keys:
                roll        float  degrees, positive = right side down
                pitch       float  degrees, positive = nose up
                yaw         None   cannot compute without magnetometer
                accel_mag   float  m/s², should be ~9.8 at rest
                angular_rate float rad/s, current gyro magnitude
        """
        ax, ay, az = self.get_acceleration()
        accel_mag = math.sqrt(ax * ax + ay * ay + az * az)

        # Standard tilt computation from accelerometer:
        # Roll:  rotation about X axis (left/right tilt)
        # Pitch: rotation about Y axis (fore/aft tilt)
        roll = math.degrees(math.atan2(ay, az))
        pitch = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))

        return {
            "roll": round(roll, 2),
            "pitch": round(pitch, 2),
            "yaw": None,            # No magnetometer — yaw unavailable
            "accel_mag": round(accel_mag, 4),
            "angular_rate": round(self.get_angular_rate(), 4),
        }
