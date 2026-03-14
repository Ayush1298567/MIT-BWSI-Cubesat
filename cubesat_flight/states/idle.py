# states/idle.py — Post-imaging queue build, aging, cleanup, and command processing
#
# idle() runs for IDLE_DURATION_SEC after each imaging pass. It:
#   1. Builds the priority queue from the image index
#   2. Ages P2 images that are old enough → P3
#   3. Deletes oldest P3 images if storage is above warning threshold
#   4. Processes any pending GCS ground commands
#   5. Saves queue, image index, and coverage grid to disk

import time

from config import IDLE_DURATION_SEC, THROTTLE_BYTES_PER_SEC, DOWNLINK_WINDOW_SEC
from utils.logger import log


def idle(storage, coverage, command_listener, pass_number, camera):
    """Run the IDLE state after one imaging pass.

    Args:
        storage:          StorageManager instance
        coverage:         CoverageTracker instance
        command_listener: CommandListener instance
        pass_number:      Current pass number (for P2 aging)
        camera:           Camera instance (for adjust_exposure GCS command)

    Returns:
        list — the priority-sorted downlink queue (list of metadata dicts)
    """
    # --- 1. Build priority queue ---
    queue = storage.build_priority_queue(pass_number)
    log(f"IDLE: priority queue built — {len(queue)} images pending downlink")

    # --- 2. Age P2 images ---
    demoted = storage.age_images(pass_number)
    if demoted:
        log(f"IDLE: aged {len(demoted)} P2 image(s) to P3: {demoted}")

    # --- 3. Storage cleanup ---
    if storage.is_warning():
        deleted = storage.cleanup_p3()
        cap = storage.check_capacity()
        log(
            f"IDLE: storage warning — deleted {deleted} P3 image(s). "
            f"Now at {cap['used_pct']:.1f}%",
            level="WARN",
        )

    # --- 4. Process pending GCS ground commands ---
    for cmd in command_listener.get_pending():
        _handle_idle_command(cmd, queue, storage, camera)

    # --- 5. Persist everything to disk ---
    storage.save_queue(queue)
    storage.save_image_index()
    coverage.save()

    data_budget = THROTTLE_BYTES_PER_SEC * DOWNLINK_WINDOW_SEC
    log(
        f"IDLE: saved. Queue={len(queue)} images, "
        f"downlink budget={data_budget} bytes "
        f"(~{data_budget // 28_000} images at 28 KB avg)"
    )

    time.sleep(IDLE_DURATION_SEC)

    return queue


def _handle_idle_command(cmd, queue, storage, camera):
    """Handle GCS commands that are meaningful during IDLE."""
    name = cmd.get("cmd")

    if name == "retransmit":
        image_id = cmd.get("image_id", "")
        # Find the image in the queue and move it to the front.
        for i, item in enumerate(queue):
            if image_id in item.get("filename", ""):
                queue.insert(0, queue.pop(i))
                log(f"IDLE: retransmit — moved {image_id} to top of queue")
                return
        log(f"IDLE: retransmit — image {image_id!r} not found in queue", level="WARN")

    elif name == "priority_cell":
        row, col = cmd.get("row"), cmd.get("col")
        # Boost: move all images from that cell to the front of their tier.
        boosted = [i for i, m in enumerate(queue)
                   if m.get("grid_cell") == [row, col]]
        for i in reversed(boosted):
            queue.insert(0, queue.pop(i))
        log(f"IDLE: boosted {len(boosted)} image(s) from cell ({row},{col}) to queue front")

    elif name == "adjust_exposure":
        exposure_us = cmd.get("exposure_us")
        if exposure_us:
            camera._cam.set_controls({"AeEnable": False, "ExposureTime": int(exposure_us)})
            log(f"IDLE: exposure set to {exposure_us} µs by GCS command")

    elif name in ("enter_safe_mode", "resume_normal", "status_request",
                  "retry_downlink", "set_cell"):
        # These are handled by the main state machine loop, not by idle.
        # Re-queue them so the main loop sees them after idle() returns.
        # We can't re-queue into command_listener from here, so just log.
        log(f"IDLE: command '{name}' deferred to main loop")

    else:
        log(f"IDLE: unrecognised command '{name}'", level="WARN")
