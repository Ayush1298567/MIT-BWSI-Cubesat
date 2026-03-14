# utils/thermal.py — CPU temperature monitoring via sysfs
#
# Reads /sys/class/thermal/thermal_zone0/temp every THERMAL_CHECK_INTERVAL_SEC
# in a background daemon thread. The file stores millidegrees Celsius as an int,
# so the raw value is divided by 1000 to get °C.
#
# Warning threshold: CPU_TEMP_WARNING_C (70°C) — imaging loop doubles interval
# Critical threshold: CPU_TEMP_CRITICAL_C (80°C) — enter safe mode

import threading
import time

from config import CPU_TEMP_CRITICAL_C, CPU_TEMP_WARNING_C, THERMAL_CHECK_INTERVAL_SEC

_THERMAL_PATH = "/sys/class/thermal/thermal_zone0/temp"


class Thermal:
    def __init__(self):
        self._temp_c = 0.0
        self._lock = threading.Lock()
        self._thread = None

    def start_monitoring(self):
        """Start the background daemon thread. Call once at boot."""
        self._thread = threading.Thread(
            target=self._monitor_loop, daemon=True, name="ThermalMonitor"
        )
        self._thread.start()

    def get_cpu_temp(self):
        """Return the most recently read CPU temperature in °C."""
        with self._lock:
            return self._temp_c

    def is_warning(self):
        """Return True if CPU temp exceeds CPU_TEMP_WARNING_C (70°C)."""
        return self.get_cpu_temp() > CPU_TEMP_WARNING_C

    def is_critical(self):
        """Return True if CPU temp exceeds CPU_TEMP_CRITICAL_C (80°C)."""
        return self.get_cpu_temp() > CPU_TEMP_CRITICAL_C

    # ------------------------------------------------------------------
    # Background thread
    # ------------------------------------------------------------------

    def _monitor_loop(self):
        while True:
            try:
                with open(_THERMAL_PATH, "r") as f:
                    raw = int(f.read().strip())
                temp = raw / 1000.0
                with self._lock:
                    self._temp_c = temp
            except OSError:
                pass  # sysfs temporarily unavailable — keep last reading
            time.sleep(THERMAL_CHECK_INTERVAL_SEC)
