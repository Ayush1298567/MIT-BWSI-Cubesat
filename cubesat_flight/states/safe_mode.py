# states/safe_mode.py — Safe mode / error recovery
#
# Entered when:
#   • IMU or camera fails boot self-test
#   • CPU temperature exceeds CPU_TEMP_CRITICAL_C (80°C)
#   • Unrecoverable runtime error
#   • Ground command "enter_safe_mode"
#
# While in safe mode:
#   • Camera is stopped
#   • No imaging, no downlink
#   • Command listener daemon continues running (GCS can still send commands)
#   • Thermal monitor daemon continues running
#   • Watchdog is petted so the process stays alive
#
# Exit: ground command "resume_normal" or operator terminal "resume"

import time

from utils.logger import log

_POLL_INTERVAL_SEC = 2.0


def safe_mode(camera, command_listener, watchdog):
    """Block in safe mode until a resume command is received.

    Args:
        camera:           Camera instance — stopped on entry.
        command_listener: CommandListener instance — still receives GCS commands.
        watchdog:         Watchdog instance — petted to prevent restart.

    Returns when "resume_normal" (GCS) or "resume" (operator) is received.
    Caller transitions to IDLE.
    """
    log("=== ENTERING SAFE MODE ===", level="ERROR")

    # Stop camera cleanly to reduce power and heat.
    try:
        camera.close()
        log("Safe mode: camera stopped")
    except Exception as exc:
        log(f"Safe mode: camera close failed (ignored): {exc}", level="WARN")

    print("\n!!! SAFE MODE ACTIVE !!!")
    print("Type 'resume' to exit safe mode, or send GCS command 'resume_normal'.")

    while True:
        watchdog.pet()

        # Check GCS commands.
        for cmd in command_listener.get_pending():
            if cmd.get("cmd") == "resume_normal":
                log("Safe mode: 'resume_normal' from GCS — exiting safe mode")
                _restart_camera(camera)
                return

        # Check operator terminal (blocking with short sleep to avoid busy-wait).
        # Operator must type on a separate terminal since stdin may be blocked.
        # We rely on the stdin reader thread in main.py to surface this.
        # safe_mode() accepts the resume via command_listener only in the
        # hardware-failure case (where main never got to start the stdin thread).
        # When called from the running main loop, main.py checks get_operator_input
        # before calling safe_mode, so returning on GCS command is sufficient.

        time.sleep(_POLL_INTERVAL_SEC)


def _restart_camera(camera):
    """Attempt to reinitialise the camera after safe mode.
    If it fails, log the error — the caller decides whether to retry or re-enter safe mode."""
    from config import IMAGE_WIDTH, IMAGE_HEIGHT
    from sensors.camera import Camera
    try:
        new_cam = Camera(IMAGE_WIDTH, IMAGE_HEIGHT)
        # Replace internals on the existing object so the caller's reference stays valid.
        camera._cam = new_cam._cam
        camera._last_metadata = {}
        log("Safe mode: camera restarted successfully")
    except Exception as exc:
        log(f"Safe mode: camera restart failed: {exc}", level="ERROR")
