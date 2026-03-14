# utils/logger.py — Flight software logger
#
# Named logger.py (NOT logging.py) to avoid clashing with the Python stdlib
# logging module.
#
# Log format:  {ISO_timestamp} [{STATE}] [{LEVEL}] {message}
# Example:     2026-03-15T14:45:00Z [IMAGING] [INFO] STABLE + NADIR — capturing
#
# Appends to a rotating log file in LOG_DIR. Keeps the last 50 log lines in
# memory so build_telemetry() can include recent_log in telemetry packets.
#
# Module-level singleton + convenience function:
#   from utils.logger import log
#   log("something happened")
#   log("debug detail", level="DEBUG")

import collections
import os
import threading
from datetime import datetime, timezone

from config import LOG_DIR

_LOG_FILE = os.path.join(LOG_DIR, "flight.log")
_MAX_LOG_FILE_BYTES = 5 * 1024 * 1024  # 5 MB before rotation
_MEMORY_LINES = 50


class Logger:
    def __init__(self):
        self._state = "BOOT"
        self._recent = collections.deque(maxlen=_MEMORY_LINES)
        self._lock = threading.Lock()
        os.makedirs(LOG_DIR, exist_ok=True)
        self._file = open(_LOG_FILE, "a", buffering=1)  # line-buffered

    def set_state(self, state):
        """Update the state label prepended to every subsequent log line."""
        self._state = state.upper()

    def info(self, message):
        self._write("INFO", message)

    def debug(self, message):
        self._write("DEBUG", message)

    def warn(self, message):
        self._write("WARN", message)

    def error(self, message):
        self._write("ERROR", message)

    def get_recent(self):
        """Return a list of the most recent _MEMORY_LINES log lines (oldest first)."""
        with self._lock:
            return list(self._recent)

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _write(self, level, message):
        timestamp = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
        line = f"{timestamp} [{self._state}] [{level}] {message}"
        with self._lock:
            self._recent.append(line)
            try:
                self._file.write(line + "\n")
                self._rotate_if_needed()
            except OSError:
                pass  # Don't crash the flight software on a logging error

    def _rotate_if_needed(self):
        """Rotate the log file if it exceeds _MAX_LOG_FILE_BYTES."""
        try:
            if os.path.getsize(_LOG_FILE) < _MAX_LOG_FILE_BYTES:
                return
            self._file.close()
            rotated = _LOG_FILE + ".1"
            if os.path.exists(rotated):
                os.remove(rotated)
            os.rename(_LOG_FILE, rotated)
            self._file = open(_LOG_FILE, "a", buffering=1)
        except OSError:
            pass


# ---------------------------------------------------------------------------
# Module-level singleton and convenience function
# ---------------------------------------------------------------------------

_instance = Logger()


def log(message, level="INFO"):
    """Write a log line using the module-level Logger singleton.

    Args:
        message: The log message string.
        level:   "DEBUG", "INFO", "WARN", or "ERROR". Default "INFO".
    """
    _instance._write(level, message)


def set_state(state):
    """Update the state label on the module-level singleton."""
    _instance.set_state(state)


def get_recent():
    """Return recent log lines from the module-level singleton."""
    return _instance.get_recent()


def get_logger():
    """Return the module-level Logger instance (for direct method access)."""
    return _instance
