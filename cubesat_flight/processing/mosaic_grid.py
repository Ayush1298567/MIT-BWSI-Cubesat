# processing/mosaic_grid.py — Dual-resolution cost grid for route planning
#
# Maintains two parallel grids over the mosaic image:
#   Coarse grid — MOSAIC_GRID_CELL_PX (80px) cells for backward compatibility
#   Fine grid   — SEG_GRID_CELL_PX (20px) cells for pixel-level route planning
#
# Each cell stores a traversal cost (for A*) and a hazard flag (for display).
# The fine grid is populated by apply_segmentation_mask() from PixelSegmenter
# output. The coarse grid is populated by apply_classification() from the
# original per-image hazard classification.

import numpy as np

from config import MOSAIC_GRID_CELL_PX, SEG_GRID_CELL_PX, SEG_COST_MAP


class MosaicGrid:
    """Dual-resolution cost grid over a mosaic image."""

    def __init__(self):
        # Mosaic dimensions in pixels (updated as images are stitched)
        self._mosaic_w = 0
        self._mosaic_h = 0

        # Coarse grid (80px cells)
        self._coarse_rows = 0
        self._coarse_cols = 0
        self._coarse_cost_grid = None    # float32
        self._coarse_hazard_grid = None  # uint8

        # Fine grid (20px cells)
        self._fine_rows = 0
        self._fine_cols = 0
        self._fine_cost_grid = None      # float32
        self._fine_hazard_grid = None    # uint8 (dominant label per cell)

    def update_from_mosaic(self, mosaic_w, mosaic_h):
        """Resize both grids to match current mosaic dimensions.

        Called whenever the mosaic grows (new image stitched).
        Preserves existing cell values where the grid overlaps.
        """
        if mosaic_w == self._mosaic_w and mosaic_h == self._mosaic_h:
            return

        self._mosaic_w = mosaic_w
        self._mosaic_h = mosaic_h

        # Coarse grid
        new_cr = max(1, (mosaic_h + MOSAIC_GRID_CELL_PX - 1) // MOSAIC_GRID_CELL_PX)
        new_cc = max(1, (mosaic_w + MOSAIC_GRID_CELL_PX - 1) // MOSAIC_GRID_CELL_PX)
        self._coarse_cost_grid = self._resize_grid(
            self._coarse_cost_grid, self._coarse_rows, self._coarse_cols,
            new_cr, new_cc, default=1.0, dtype=np.float32
        )
        self._coarse_hazard_grid = self._resize_grid(
            self._coarse_hazard_grid, self._coarse_rows, self._coarse_cols,
            new_cr, new_cc, default=0, dtype=np.uint8
        )
        self._coarse_rows = new_cr
        self._coarse_cols = new_cc

        # Fine grid
        new_fr = max(1, (mosaic_h + SEG_GRID_CELL_PX - 1) // SEG_GRID_CELL_PX)
        new_fc = max(1, (mosaic_w + SEG_GRID_CELL_PX - 1) // SEG_GRID_CELL_PX)
        self._fine_cost_grid = self._resize_grid(
            self._fine_cost_grid, self._fine_rows, self._fine_cols,
            new_fr, new_fc, default=1.0, dtype=np.float32
        )
        self._fine_hazard_grid = self._resize_grid(
            self._fine_hazard_grid, self._fine_rows, self._fine_cols,
            new_fr, new_fc, default=0, dtype=np.uint8
        )
        self._fine_rows = new_fr
        self._fine_cols = new_fc

    def apply_classification(self, mosaic_bbox, hazard_class, cost, confidence=1.0):
        """Apply a single hazard classification to coarse grid cells.

        This is the original coarse approach: one class per image region.

        Args:
            mosaic_bbox: (x, y, w, h) in mosaic pixel coords.
            hazard_class: str label (e.g. "crater", "safe").
            cost: float traversal cost for A*.
            confidence: float 0-1 detection confidence.
        """
        if self._coarse_cost_grid is None:
            return

        x, y, w, h = mosaic_bbox
        r0 = max(0, y // MOSAIC_GRID_CELL_PX)
        c0 = max(0, x // MOSAIC_GRID_CELL_PX)
        r1 = min(self._coarse_rows, (y + h + MOSAIC_GRID_CELL_PX - 1) // MOSAIC_GRID_CELL_PX)
        c1 = min(self._coarse_cols, (x + w + MOSAIC_GRID_CELL_PX - 1) // MOSAIC_GRID_CELL_PX)

        weighted_cost = 1.0 + (cost - 1.0) * confidence
        is_hazard = 1 if cost > 5 else 0

        for r in range(r0, r1):
            for c in range(c0, c1):
                self._coarse_cost_grid[r, c] = max(
                    self._coarse_cost_grid[r, c], weighted_cost
                )
                if is_hazard:
                    self._coarse_hazard_grid[r, c] = 1

    def apply_segmentation_mask(self, mosaic_bbox, label_map, cost_map=None,
                                confidence=1.0):
        """Project a pixel-level label map onto the fine grid.

        For each fine grid cell overlapping the bbox, samples the label_map pixels
        and assigns the worst label covering >=25% of the cell (conservative).
        Otherwise uses majority label.

        Args:
            mosaic_bbox: (x, y, w, h) in mosaic pixel coords — where the image
                         sits in the mosaic.
            label_map: np.ndarray (img_h, img_w) uint8 from PixelSegmenter.
            cost_map: dict label→cost, defaults to SEG_COST_MAP.
            confidence: float 0-1 overall confidence.
        """
        if self._fine_cost_grid is None:
            return

        if cost_map is None:
            cost_map = SEG_COST_MAP

        mx, my, mw, mh = mosaic_bbox
        img_h, img_w = label_map.shape[:2]

        # Map fine grid cell range
        fr0 = max(0, my // SEG_GRID_CELL_PX)
        fc0 = max(0, mx // SEG_GRID_CELL_PX)
        fr1 = min(self._fine_rows, (my + mh + SEG_GRID_CELL_PX - 1) // SEG_GRID_CELL_PX)
        fc1 = min(self._fine_cols, (mw + mx + SEG_GRID_CELL_PX - 1) // SEG_GRID_CELL_PX)

        hazard_threshold = 0.25  # 25% coverage triggers worst-label assignment

        for r in range(fr0, fr1):
            for c in range(fc0, fc1):
                # Pixel range in mosaic coords
                py0 = r * SEG_GRID_CELL_PX
                px0 = c * SEG_GRID_CELL_PX
                py1 = py0 + SEG_GRID_CELL_PX
                px1 = px0 + SEG_GRID_CELL_PX

                # Convert to image-local coords
                iy0 = max(0, py0 - my)
                ix0 = max(0, px0 - mx)
                iy1 = min(img_h, py1 - my)
                ix1 = min(img_w, px1 - mx)

                if iy1 <= iy0 or ix1 <= ix0:
                    continue

                cell_pixels = label_map[iy0:iy1, ix0:ix1]
                total = cell_pixels.size
                if total == 0:
                    continue

                # Find worst (highest cost) label with >= 25% coverage
                unique, counts = np.unique(cell_pixels, return_counts=True)
                assigned_label = None
                assigned_cost = 0

                for lbl, cnt in zip(unique, counts):
                    lbl_cost = cost_map.get(int(lbl), 1)
                    fraction = cnt / total
                    if lbl_cost > assigned_cost and fraction >= hazard_threshold:
                        assigned_cost = lbl_cost
                        assigned_label = int(lbl)

                # If no hazard meets threshold, use majority label
                if assigned_label is None:
                    majority_idx = np.argmax(counts)
                    assigned_label = int(unique[majority_idx])
                    assigned_cost = cost_map.get(assigned_label, 1)

                weighted_cost = 1.0 + (assigned_cost - 1.0) * confidence
                self._fine_cost_grid[r, c] = max(
                    self._fine_cost_grid[r, c], weighted_cost
                )
                self._fine_hazard_grid[r, c] = assigned_label

    # ------------------------------------------------------------------
    # Accessors
    # ------------------------------------------------------------------

    def get_coarse_cost_grid(self):
        return self._coarse_cost_grid

    def get_coarse_hazard_grid(self):
        return self._coarse_hazard_grid

    def get_fine_cost_grid(self):
        return self._fine_cost_grid

    def get_fine_hazard_grid(self):
        return self._fine_hazard_grid

    @property
    def fine_rows(self):
        return self._fine_rows

    @property
    def fine_cols(self):
        return self._fine_cols

    @property
    def coarse_rows(self):
        return self._coarse_rows

    @property
    def coarse_cols(self):
        return self._coarse_cols

    @property
    def mosaic_w(self):
        return self._mosaic_w

    @property
    def mosaic_h(self):
        return self._mosaic_h

    def mosaic_px_to_fine_grid(self, px_x, px_y):
        """Convert mosaic pixel coords to fine grid (row, col)."""
        return (px_y // SEG_GRID_CELL_PX, px_x // SEG_GRID_CELL_PX)

    def mosaic_px_to_coarse_grid(self, px_x, px_y):
        """Convert mosaic pixel coords to coarse grid (row, col)."""
        return (px_y // MOSAIC_GRID_CELL_PX, px_x // MOSAIC_GRID_CELL_PX)

    def fine_grid_to_mosaic_px(self, row, col):
        """Convert fine grid (row, col) to mosaic pixel center."""
        cx = col * SEG_GRID_CELL_PX + SEG_GRID_CELL_PX // 2
        cy = row * SEG_GRID_CELL_PX + SEG_GRID_CELL_PX // 2
        return (cx, cy)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _resize_grid(old_grid, old_rows, old_cols, new_rows, new_cols,
                     default, dtype):
        """Create a new grid, copying overlapping region from old grid."""
        new_grid = np.full((new_rows, new_cols), default, dtype=dtype)
        if old_grid is not None:
            copy_r = min(old_rows, new_rows)
            copy_c = min(old_cols, new_cols)
            new_grid[:copy_r, :copy_c] = old_grid[:copy_r, :copy_c]
        return new_grid
