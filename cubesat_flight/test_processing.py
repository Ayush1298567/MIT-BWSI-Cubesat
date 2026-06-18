"""Hardware-independent regression tests for mapping and route planning."""

import os
import sys
import unittest

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

import config
from processing.mosaic_grid import MosaicGrid
from processing.pixel_segmenter import BOULDER, SAND
from processing.route_planner import RoutePlanner


class MosaicGridTest(unittest.TestCase):
    def test_segmentation_projects_to_offset_bbox(self):
        grid = MosaicGrid()
        grid.update_from_mosaic(200, 200)

        labels = np.full((40, 60), SAND, dtype=np.uint8)
        labels[:, 40:] = BOULDER
        grid.apply_segmentation_mask((100, 120, 60, 40), labels)

        hazard_grid = grid.get_fine_hazard_grid()
        self.assertEqual(hazard_grid[6, 7], BOULDER)
        self.assertEqual(hazard_grid[6, 5], SAND)


class RoutePlannerTest(unittest.TestCase):
    def test_astar_routes_around_impassable_cells(self):
        old_enabled = config.SEG_ENABLED
        config.SEG_ENABLED = True
        try:
            grid = MosaicGrid()
            grid.update_from_mosaic(100, 100)
            costs = grid.get_fine_cost_grid()
            costs[:, 2] = 999
            costs[4, 2] = 1

            result = RoutePlanner().plan_route(grid, (5, 5), (85, 5))
            self.assertTrue(result["success"])
            self.assertIn((4, 2), result["path_grid"])
        finally:
            config.SEG_ENABLED = old_enabled


if __name__ == "__main__":
    unittest.main(verbosity=2)
