# states/downlink.py — Throttled TCP downlink to ground station
#
# Sends a telemetry packet first, then pops images from the priority queue
# highest-priority first. Transfer is throttled to THROTTLE_BYTES_PER_SEC
# (1200 B/s) to simulate the UHF flight link — a 28 KB image takes ~23 seconds.
#
# Data budget: THROTTLE_BYTES_PER_SEC × DOWNLINK_WINDOW_SEC = 72,000 bytes.
# At ~28 KB per image, roughly 2 images per window.
#
# Error handling:
#   NACK      → retry up to MAX_RETRIES_PER_IMAGE, then mark corrupt and skip
#   Socket error → log, stop downlink for this pass; image stays in queue
#   GCSUnreachableError → raise to caller so gcs_suspended flag can be set

import os
import time

from comms.transfer import GCSUnreachableError
from config import (
    DOWNLINK_WINDOW_SEC,
    IMAGE_DIR,
    MAX_RETRIES_PER_IMAGE,
    THROTTLE_BYTES_PER_SEC,
)
from utils.logger import log


def downlink(transfer, storage, queue, telemetry_dict, watchdog):
    """Run one downlink window.

    Args:
        transfer:       Transfer instance (TCP client)
        storage:        StorageManager instance
        queue:          Priority-sorted list of image metadata dicts (modified in place)
        telemetry_dict: Pre-built telemetry dict from build_telemetry()
        watchdog:       Watchdog instance — petted between image sends

    Returns:
        dict with keys:
            images_sent      int   — number of images ACKed this window
            bytes_sent       int   — total bytes sent (telemetry + images)
            remaining_queue  list  — queue items not yet sent
            gcs_suspended    bool  — True if GCSUnreachableError was raised
    """
    data_budget = THROTTLE_BYTES_PER_SEC * DOWNLINK_WINDOW_SEC  # 72,000 bytes
    bytes_sent = 0
    images_sent = 0

    try:
        transfer.connect()
    except GCSUnreachableError as exc:
        log(str(exc), level="ERROR")
        return {
            "images_sent": 0,
            "bytes_sent": 0,
            "remaining_queue": queue,
            "gcs_suspended": True,
        }
    except OSError as exc:
        log(f"GCS connect failed: {exc}", level="WARN")
        return {
            "images_sent": 0,
            "bytes_sent": 0,
            "remaining_queue": queue,
            "gcs_suspended": False,
        }

    # Reset per-window bytes counter on the Transfer object.
    transfer.bytes_sent = 0

    # --- Send telemetry first ---
    log("Downlink: sending telemetry packet")
    telem_ok = transfer.send_telemetry(telemetry_dict)
    if telem_ok:
        bytes_sent += transfer.bytes_sent
        log(f"Downlink: telemetry sent ({transfer.bytes_sent} bytes)")
    else:
        log(f"Downlink: telemetry send failed ({transfer.last_error})", level="WARN")

    # Reset for image counting.
    transfer.bytes_sent = 0
    window_start = time.monotonic()

    # --- Send images from queue ---
    sent_indices = []   # indices into queue that were successfully sent

    i = 0
    while i < len(queue):
        watchdog.pet()

        if time.monotonic() - window_start >= DOWNLINK_WINDOW_SEC:
            log(f"Downlink: window time exhausted. {len(queue) - i} images remain.")
            break

        item = queue[i]
        file_size = item.get("file_size_bytes", 0)

        if bytes_sent + file_size > data_budget:
            log(
                f"Downlink: budget exhausted ({bytes_sent}/{data_budget} bytes). "
                f"{len(queue) - i} images remain queued."
            )
            break

        filepath = os.path.join(IMAGE_DIR, item["filename"])
        if not os.path.exists(filepath):
            log(f"Downlink: image file missing on disk: {item['filename']}", level="WARN")
            storage.mark_failed(item["filename"])
            sent_indices.append(i)  # Remove from queue (not retryable)
            i += 1
            continue

        ok = transfer.send_file(filepath, item, watchdog=watchdog)

        if ok:
            log(f"  SENT {item['filename']} ({file_size} B)")
            bytes_sent += file_size
            images_sent += 1
            storage.mark_sent(item["filename"])
            sent_indices.append(i)
            i += 1

        elif transfer.last_error == "NACK":
            item["retry_count"] = item.get("retry_count", 0) + 1
            if item["retry_count"] >= MAX_RETRIES_PER_IMAGE:
                log(
                    f"  CORRUPT {item['filename']} — "
                    f"{MAX_RETRIES_PER_IMAGE} NACKs, marking failed and skipping",
                    level="WARN",
                )
                storage.mark_failed(item["filename"])
                sent_indices.append(i)  # Remove corrupt image from queue
                i += 1
            else:
                log(
                    f"  NACK {item['filename']} — "
                    f"retry {item['retry_count']}/{MAX_RETRIES_PER_IMAGE}"
                )
                # Leave i unchanged — retry same item next iteration.

        else:
            # Socket error, timeout, or unexpected byte — stop downlink.
            log(f"  LINK FAIL ({transfer.last_error}) — stopping downlink", level="WARN")
            break

    # Build remaining queue by excluding items that were sent or failed.
    sent_set = set(sent_indices)
    remaining = [item for j, item in enumerate(queue) if j not in sent_set]

    try:
        transfer.disconnect()
    except OSError:
        pass

    log(
        f"Downlink: {images_sent} sent, {bytes_sent} bytes, "
        f"{len(remaining)} images still queued"
    )

    return {
        "images_sent": images_sent,
        "bytes_sent": bytes_sent,
        "remaining_queue": remaining,
        "gcs_suspended": False,
    }
