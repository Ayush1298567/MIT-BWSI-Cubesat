# processing/coverage.py — 8×8 coverage grid tracker
#
# Tracks which grid cells have been imaged and at what quality.
# Grid cell assignment comes exclusively from operator terminal input
# ("cell R C") — NOT from elapsed time or any trajectory map.
#
# Novelty scoring drives the priority queue:
#   1.0 — cell has never been captured
#   0.5 — cell has a capture, but quality could still be improved
#   0.1 — cell has a good-quality capture; further shots are redundant
#
# The threshold between "better possible" and "redundant" is 0.7.
# A cell scoring above 0.7 is considered well-covered.

import json
import os
from datetime import datetime, timezone

from config import GRID_ROWS, GRID_COLS

COVERAGE_FILE = "/home/cubesat/cubesat_flight/data/coverage.json"

# Quality score above which a cell is considered well-covered (redundant).
_GOOD_QUALITY_THRESHOLD = 0.7


class CoverageTracker:
    def __init__(self):
        # Grid stored as a dict keyed by "R,C" string for JSON compatibility.
        # Value is a dict with: covered, best_quality_score, best_image_filename,
        # pass_number, timestamp.
        self._grid = {}

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    def get_novelty(self, cell):
        """Return novelty score for a grid cell.

        Args:
            cell: (row, col) tuple — from operator-set current_grid_cell.

        Returns:
            1.0  cell never captured
            0.5  cell captured but best quality score < 0.7 (room to improve)
            0.1  cell captured with quality >= 0.7 (redundant)
        """
        entry = self._grid.get(self._key(cell))
        if entry is None or not entry["covered"]:
            return 1.0
        if entry["best_quality_score"] < _GOOD_QUALITY_THRESHOLD:
            return 0.5
        return 0.1

    def update(self, cell, score, filename):
        """Record a successful capture for a grid cell.
        Updates the cell's best quality score and image if this capture is better.

        Args:
            cell:     (row, col) tuple
            score:    Combined quality score from QualityGate (0.0–1.0)
            filename: Filename of the captured image (not full path)
        """
        key = self._key(cell)
        existing = self._grid.get(key)

        if existing is None or score > existing["best_quality_score"]:
            self._grid[key] = {
                "covered": True,
                "best_quality_score": round(score, 4),
                "best_image_filename": filename,
                "pass_number": None,    # filled in by caller if needed
                "timestamp": datetime.now(timezone.utc).isoformat(),
                "row": cell[0],
                "col": cell[1],
            }
        elif not existing["covered"]:
            # Edge case: entry exists from a load() but was marked not covered.
            existing["covered"] = True
            existing["best_quality_score"] = round(score, 4)
            existing["best_image_filename"] = filename
            existing["timestamp"] = datetime.now(timezone.utc).isoformat()

    def get_coverage_pct(self):
        """Return percentage of grid cells that have at least one capture."""
        total = GRID_ROWS * GRID_COLS
        covered = sum(
            1 for e in self._grid.values() if e.get("covered", False)
        )
        return round(100.0 * covered / total, 1)

    def get_summary(self):
        """Return a summary dict suitable for telemetry and logging.

        Returns:
            dict with:
                cells_filled  int    number of cells with at least one capture
                cells_total   int    total grid cells (GRID_ROWS × GRID_COLS)
                pct           float  percentage covered
                grid          dict   full cell-by-cell state keyed by "R,C"
        """
        total = GRID_ROWS * GRID_COLS
        covered = sum(
            1 for e in self._grid.values() if e.get("covered", False)
        )
        return {
            "cells_filled": covered,
            "cells_total": total,
            "pct": round(100.0 * covered / total, 1),
            "grid": dict(self._grid),
        }

    def save(self):
        """Persist current grid state to COVERAGE_FILE (JSON)."""
        os.makedirs(os.path.dirname(COVERAGE_FILE), exist_ok=True)
        with open(COVERAGE_FILE, "w") as f:
            json.dump(self._grid, f, indent=2)

    def load(self):
        """Load grid state from COVERAGE_FILE.
        Safe to call even if the file does not exist yet (starts empty)."""
        if not os.path.exists(COVERAGE_FILE):
            return
        with open(COVERAGE_FILE, "r") as f:
            self._grid = json.load(f)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _key(cell):
        """Convert (row, col) tuple to the string key used in the grid dict."""
        return f"{cell[0]},{cell[1]}"
