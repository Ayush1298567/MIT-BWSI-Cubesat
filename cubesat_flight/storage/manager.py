# storage/manager.py — Queue persistence, image index, capacity, and aging
#
# Maintains two JSON files:
#   QUEUE_FILE       — ordered list of images pending downlink
#   IMAGE_INDEX_FILE — full record of every image ever captured
#
# Priority tiers: P1 (highest) > P2 > P3.
# build_priority_queue() sorts by tier then score descending.
# age_images() demotes P2 images older than P2_AGING_PASSES passes to P3.
# cleanup_p3() deletes the oldest P3 images from disk until storage is below warning.

import json
import os
import shutil

from config import (
    IMAGE_DIR,
    IMAGE_INDEX_FILE,
    P2_AGING_PASSES,
    QUEUE_FILE,
    RECOVERY_FILE,
    STORAGE_CRITICAL_PCT,
    STORAGE_WARNING_PCT,
)

_TIER_RANK = {"P1": 0, "P2": 1, "P3": 2}


class StorageManager:

    def __init__(self):
        self._image_index = {}  # filename → metadata dict
        os.makedirs(IMAGE_DIR, exist_ok=True)
        os.makedirs(os.path.dirname(QUEUE_FILE), exist_ok=True)

    # ------------------------------------------------------------------
    # Capacity
    # ------------------------------------------------------------------

    def check_capacity(self):
        """Return current disk usage for the IMAGE_DIR partition.

        Returns:
            dict with:
                used_pct  float  percentage of disk used
                free_mb   float  free megabytes
                total_mb  float  total megabytes
        """
        usage = shutil.disk_usage(IMAGE_DIR)
        used_pct = 100.0 * usage.used / usage.total
        return {
            "used_pct": round(used_pct, 1),
            "free_mb": round(usage.free / (1024 * 1024), 1),
            "total_mb": round(usage.total / (1024 * 1024), 1),
        }

    def is_warning(self):
        return self.check_capacity()["used_pct"] > STORAGE_WARNING_PCT

    def is_critical(self):
        return self.check_capacity()["used_pct"] > STORAGE_CRITICAL_PCT

    # ------------------------------------------------------------------
    # Queue persistence
    # ------------------------------------------------------------------

    def save_queue(self, queue):
        """Persist the downlink queue list to QUEUE_FILE."""
        with open(QUEUE_FILE, "w") as f:
            json.dump(queue, f, indent=2, default=str)

    def load_queue(self):
        """Load the downlink queue from QUEUE_FILE.
        Returns an empty list if the file does not exist."""
        if not os.path.exists(QUEUE_FILE):
            return []
        with open(QUEUE_FILE, "r") as f:
            return json.load(f)

    # ------------------------------------------------------------------
    # Image index
    # ------------------------------------------------------------------

    def save_image_index(self):
        """Write the full in-memory image index to IMAGE_INDEX_FILE."""
        with open(IMAGE_INDEX_FILE, "w") as f:
            json.dump(self._image_index, f, indent=2, default=str)

    def load_image_index(self):
        """Load the image index from IMAGE_INDEX_FILE into memory.
        Safe to call even if the file does not exist yet."""
        if not os.path.exists(IMAGE_INDEX_FILE):
            self._image_index = {}
            return
        with open(IMAGE_INDEX_FILE, "r") as f:
            self._image_index = json.load(f)

    def update_image_index(self, filename, meta):
        """Add or overwrite the index entry for one image.

        Args:
            filename: Bare filename (key), e.g. "pass3_img07_20260315_144500.jpg"
            meta:     Full metadata dict from MetadataBuilder.build()
        """
        self._image_index[filename] = meta

    def mark_sent(self, filename):
        """Update downlink_status to 'sent' for a successfully transferred image."""
        if filename in self._image_index:
            self._image_index[filename]["downlink_status"] = "sent"

    def mark_failed(self, filename):
        """Update downlink_status to 'failed' and increment retry_count."""
        if filename in self._image_index:
            entry = self._image_index[filename]
            entry["downlink_status"] = "failed"
            entry["retry_count"] = entry.get("retry_count", 0) + 1

    # ------------------------------------------------------------------
    # Priority queue builder
    # ------------------------------------------------------------------

    def build_priority_queue(self, current_pass):
        """Build the ordered downlink queue from the image index.

        Includes only images with downlink_status == "pending".
        Sorts by: tier (P1 → P2 → P3) then combined_score descending.

        Args:
            current_pass: Current pass number (used by age_images, not here).

        Returns:
            list of metadata dicts, highest priority first.
        """
        pending = [
            meta for meta in self._image_index.values()
            if meta.get("downlink_status") == "pending"
            and meta.get("priority_tier") is not None
        ]

        def sort_key(m):
            tier = _TIER_RANK.get(m.get("priority_tier", "P3"), 2)
            score = m.get("quality", {}).get("combined_score", 0.0)
            return (tier, -score)

        pending.sort(key=sort_key)
        return pending

    # ------------------------------------------------------------------
    # Aging
    # ------------------------------------------------------------------

    def age_images(self, current_pass):
        """Demote P2 images from passes older than P2_AGING_PASSES to P3.

        With P2_AGING_PASSES=2 and a 3-pass demo, pass 1's P2 images are
        demoted during pass 3's IDLE phase.

        Logs which images were demoted (returns a list for the caller to log).
        """
        demoted = []
        for filename, meta in self._image_index.items():
            if meta.get("priority_tier") != "P2":
                continue
            image_pass = meta.get("pass_number", current_pass)
            if (current_pass - image_pass) > P2_AGING_PASSES:
                meta["priority_tier"] = "P3"
                demoted.append(filename)
        return demoted  # Caller logs: f"Aged {len(demoted)} P2 images to P3"

    # ------------------------------------------------------------------
    # Storage cleanup
    # ------------------------------------------------------------------

    def cleanup_p3(self):
        """Delete the oldest P3 images from disk until storage drops below warning.

        Deletion order: oldest first (by timestamp, falling back to filename sort).
        Updates the image index to mark deleted images as 'deleted'.
        Returns number of images deleted.
        """
        p3_images = [
            (fname, meta)
            for fname, meta in self._image_index.items()
            if meta.get("priority_tier") == "P3"
            and meta.get("downlink_status") == "pending"
        ]

        # Sort oldest first: by timestamp string (ISO format sorts lexicographically)
        p3_images.sort(key=lambda x: x[1].get("timestamp", x[0]))

        deleted = 0
        for filename, meta in p3_images:
            if not self.is_warning():
                break
            filepath = os.path.join(IMAGE_DIR, filename)
            # Also delete sidecar
            sidecar = filepath.replace(".jpg", "_meta.json")
            for path in (filepath, sidecar):
                if os.path.exists(path):
                    os.remove(path)
            meta["downlink_status"] = "deleted"
            deleted += 1

        return deleted

    # ------------------------------------------------------------------
    # Recovery state
    # ------------------------------------------------------------------

    def save_recovery_state(self, state_dict):
        """Persist watchdog recovery state to RECOVERY_FILE.

        Args:
            state_dict: At minimum {"pass_number": N}.
                        Caller may include any additional resumption context.
        """
        with open(RECOVERY_FILE, "w") as f:
            json.dump(state_dict, f, indent=2, default=str)

    def load_recovery_state(self):
        """Load recovery state from RECOVERY_FILE.
        Returns None if the file does not exist (clean boot)."""
        if not os.path.exists(RECOVERY_FILE):
            return None
        with open(RECOVERY_FILE, "r") as f:
            return json.load(f)
