# utils/watchdog.py — Hardware-style software watchdog
#
# The main loop must call pet() on every iteration. If pet() is not called
# within WATCHDOG_TIMEOUT_SEC (30 seconds), the watchdog saves recovery state
# to disk and restarts the process with os.execv() — equivalent to a clean reboot.
#
# On restart, main.py reads the recovery file and resumes from the last known
# pass number rather than starting fresh.

import os
import sys
import threading
import time

from config import WATCHDOG_CHECK_INTERVAL_SEC, WATCHDOG_TIMEOUT_SEC
from utils.logger import log


class Watchdog:
    def __init__(self):
        self._last_pet = time.monotonic()
        self._lock = threading.Lock()
        self._thread = None
        self._recovery_callback = None  # Called with no args before restart

    def start(self, recovery_callback=None):
        """Start the watchdog daemon thread.

        Args:
            recovery_callback: Optional callable(). Called just before the
                               process is restarted so the caller can flush
                               state to disk. Typically:
                                   lambda: storage.save_recovery_state({...})
        """
        self._recovery_callback = recovery_callback
        self._last_pet = time.monotonic()
        self._thread = threading.Thread(
            target=self._watch_loop, daemon=True, name="Watchdog"
        )
        self._thread.start()

    def pet(self):
        """Reset the watchdog timer. Must be called regularly by the main loop."""
        with self._lock:
            self._last_pet = time.monotonic()

    # ------------------------------------------------------------------
    # Background thread
    # ------------------------------------------------------------------

    def _watch_loop(self):
        while True:
            time.sleep(WATCHDOG_CHECK_INTERVAL_SEC)
            with self._lock:
                elapsed = time.monotonic() - self._last_pet
            if elapsed > WATCHDOG_TIMEOUT_SEC:
                self._trigger()

    def _trigger(self):
        """Watchdog fired — save state and restart the process."""
        log(
            f"WATCHDOG TRIGGERED — no pet for >{WATCHDOG_TIMEOUT_SEC}s. "
            "Saving recovery state and restarting.",
            level="ERROR",
        )
        if self._recovery_callback is not None:
            try:
                self._recovery_callback()
            except Exception as exc:
                log(f"Recovery callback failed: {exc}", level="ERROR")

        # Replace the current process with a fresh copy.
        # sys.argv[0] is main.py; sys.executable is the Python interpreter.
        os.execv(sys.executable, [sys.executable] + sys.argv)
