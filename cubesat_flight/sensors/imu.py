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
        """Return angle in degrees between the acceleration vector and the -Z axis.
        When pointing straight down (nadir), this is 0°.
        When held sideways, this approaches 90°.
        Computed as arccos(dot(accel_normalized, [0, 0, -1]))
                   = arccos(-az / |accel|)."""
        ax, ay, az = self.get_acceleration()
        accel_mag = math.sqrt(ax * ax + ay * ay + az * az)
        if accel_mag < 1e-6:
            return 90.0  # degenerate — treat as off-nadir
        # Dot product with (0, 0, -1) is just -az
        cos_angle = -az / accel_mag
        # Clamp to [-1, 1] to guard against floating-point rounding
        cos_angle = max(-1.0, min(1.0, cos_angle))
        return math.degrees(math.acos(cos_angle))

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
