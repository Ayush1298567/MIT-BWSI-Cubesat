# processing/pipeline.py — Image processing pipeline
#
# Orchestrates the full processing chain for each captured image:
#   1. Quality gate (already run in imaging.py — results passed in)
#   2. Shadow detection (simple brightness threshold)
#   3. YOLO object detection (if model available)
#   4. Pixel segmentation (combines shadow + YOLO → label map)
#   5. Cost grid update (projects label map onto fine grid)
#   6. Route planning (A* on cost grid)
#
# The pipeline is designed to run on the Pi 4 within the 30s budget between
# image captures. Segmentation adds ~500ms per image (no GPU needed).

import os
import time

import cv2
import numpy as np

import config
from processing.mosaic_grid import MosaicGrid
from processing.pixel_segmenter import PixelSegmenter
from processing.route_planner import RoutePlanner
from utils.logger import log

# Shadow detection threshold — pixels below this brightness are shadow
_SHADOW_BRIGHTNESS_THRESHOLD = 60


class ProcessingPipeline:
    """Orchestrates per-image processing from capture to route planning."""

    def __init__(self):
        self._mosaic_grid = MosaicGrid()
        self._pixel_segmenter = PixelSegmenter()
        self._route_planner = RoutePlanner()

        # Track mosaic dimensions (grow as images are placed)
        self._mosaic_w = 0
        self._mosaic_h = 0

        # Latest route result (for dashboard queries)
        self._last_route = None

    @property
    def mosaic_grid(self):
        return self._mosaic_grid

    @property
    def route_planner(self):
        return self._route_planner

    @property
    def last_route(self):
        return self._last_route

    def process_image(self, image_path, mosaic_bbox, yolo_detections=None):
        """Run the full processing pipeline on one captured image.

        Args:
            image_path: Path to the JPEG image on disk.
            mosaic_bbox: (x, y, w, h) — where this image sits in the mosaic
                         coordinate system (in pixels).
            yolo_detections: Optional list of YOLO detection dicts, each with:
                - "class": str ("crater", "boulder", "plain")
                - "bbox": [x1, y1, x2, y2] in image pixel coords
                - "confidence": float 0-1
                If None, only shadow detection is used.

        Returns:
            dict with processing results:
                shadow_pct:    float — percentage of image that is shadow
                label_map:     np.ndarray or None — pixel segmentation (if SEG_ENABLED)
                seg_time_ms:   float — segmentation time in milliseconds
                grid_updated:  bool — whether the cost grid was updated
        """
        t0 = time.monotonic()
        result = {
            "shadow_pct": 0.0,
            "label_map": None,
            "seg_time_ms": 0.0,
            "grid_updated": False,
        }

        # Read the image
        img = cv2.imread(image_path)
        if img is None:
            log(f"Pipeline: cannot read {image_path}", level="WARN")
            return result

        h, w = img.shape[:2]

        # Grow mosaic to fit this image's bbox
        mx, my, mw, mh = mosaic_bbox
        new_mosaic_w = max(self._mosaic_w, mx + mw)
        new_mosaic_h = max(self._mosaic_h, my + mh)
        if new_mosaic_w != self._mosaic_w or new_mosaic_h != self._mosaic_h:
            self._mosaic_w = new_mosaic_w
            self._mosaic_h = new_mosaic_h
            self._mosaic_grid.update_from_mosaic(self._mosaic_w, self._mosaic_h)

        # --- Shadow detection ---
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        shadow_mask = (gray < _SHADOW_BRIGHTNESS_THRESHOLD).astype(np.uint8) * 255
        shadow_pct = 100.0 * np.count_nonzero(shadow_mask) / shadow_mask.size
        result["shadow_pct"] = round(shadow_pct, 1)

        # --- Pixel segmentation ---
        if config.SEG_ENABLED:
            t_seg = time.monotonic()
            label_map = self._pixel_segmenter.segment(
                image_path, shadow_mask, yolo_detections
            )
            seg_ms = (time.monotonic() - t_seg) * 1000
            result["label_map"] = label_map
            result["seg_time_ms"] = round(seg_ms, 1)

            # Project onto fine grid
            self._mosaic_grid.apply_segmentation_mask(
                mosaic_bbox, label_map, config.SEG_COST_MAP, confidence=1.0
            )
            result["grid_updated"] = True

            log(f"Pipeline: segmented {os.path.basename(image_path)} "
                f"shadow={shadow_pct:.0f}% seg={seg_ms:.0f}ms")
        else:
            # Fallback: coarse classification only
            hazard_class = "shadow" if shadow_pct > 30 else "safe"
            cost = 15.0 if shadow_pct > 30 else 1.0
            self._mosaic_grid.apply_classification(
                mosaic_bbox, hazard_class, cost
            )
            result["grid_updated"] = True
            log(f"Pipeline: classified {os.path.basename(image_path)} "
                f"as {hazard_class} (shadow={shadow_pct:.0f}%)")

        total_ms = (time.monotonic() - t0) * 1000
        log(f"Pipeline: total processing {total_ms:.0f}ms")

        return result

    def plan_route(self, start_px, goal_px):
        """Plan a route between two mosaic pixel coordinates.

        Args:
            start_px: (x, y) in mosaic pixel coords.
            goal_px: (x, y) in mosaic pixel coords.

        Returns:
            Route result dict from RoutePlanner.plan_route().
        """
        route = self._route_planner.plan_route(
            self._mosaic_grid, start_px, goal_px
        )
        self._last_route = route

        if route["success"]:
            log(f"Route planned: {len(route['path_grid'])} steps, "
                f"{route['distance_cm']:.1f}cm, cost={route['cost']:.1f}")
        else:
            log("Route planning failed — no path found", level="WARN")

        return route

    def get_segmentation_overlay(self):
        """Generate a color-coded segmentation overlay for the mosaic.

        Returns:
            np.ndarray (mosaic_h, mosaic_w, 4) BGRA — transparent overlay
            where each pixel is colored by its fine-grid label.
            Returns None if no segmentation data available.
        """
        hazard_grid = self._mosaic_grid.get_fine_hazard_grid()
        if hazard_grid is None:
            return None

        from processing.pixel_segmenter import LABEL_COLORS

        rows, cols = hazard_grid.shape
        # Build a small image at grid resolution, then upscale
        grid_img = np.zeros((rows, cols, 4), dtype=np.uint8)

        for label_val, bgr_color in LABEL_COLORS.items():
            mask = (hazard_grid == label_val)
            grid_img[mask, 0] = bgr_color[0]  # B
            grid_img[mask, 1] = bgr_color[1]  # G
            grid_img[mask, 2] = bgr_color[2]  # R
            grid_img[mask, 3] = 160 if label_val > 0 else 0  # Alpha (transparent for unsurveyed)

        # Upscale to mosaic pixel dimensions using nearest-neighbor
        overlay = cv2.resize(
            grid_img, (self._mosaic_w, self._mosaic_h),
            interpolation=cv2.INTER_NEAREST
        )
        return overlay
