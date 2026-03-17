# processing/route_planner.py — A* route planner on cost grid
#
# Plans shortest safe path on either the coarse or fine cost grid.
# When SEG_ENABLED, uses the fine grid (20px cells) for high-resolution
# routing that curves around individual hazards.

import heapq
import math
import os

import cv2
import numpy as np

import config

ROUTE_VIS_DIR = "/home/cubesat/cubesat_flight/data/processed/route_maps"

# Cost above which a cell is considered impassable
_IMPASSABLE_COST = 500


class RoutePlanner:
    """A* pathfinder on the mosaic cost grid."""

    def plan_route(self, mosaic_grid, start_px, goal_px):
        """Find optimal route between two mosaic pixel coordinates.

        Args:
            mosaic_grid: MosaicGrid instance with populated cost grids.
            start_px: (x, y) start position in mosaic pixel coords.
            goal_px: (x, y) goal position in mosaic pixel coords.

        Returns:
            dict with:
                path_grid:    list of (row, col) grid cells
                path_mosaic:  list of (x, y) mosaic pixel coords (cell centers)
                distance_cm:  estimated path length in cm
                cost:         total traversal cost
                cell_px:      grid cell size used (for visualization)
                success:      bool
        """
        if config.SEG_ENABLED:
            cost_grid = mosaic_grid.get_fine_cost_grid()
            cell_px = config.SEG_GRID_CELL_PX
            start_rc = mosaic_grid.mosaic_px_to_fine_grid(*start_px)
            goal_rc = mosaic_grid.mosaic_px_to_fine_grid(*goal_px)
        else:
            cost_grid = mosaic_grid.get_coarse_cost_grid()
            cell_px = config.MOSAIC_GRID_CELL_PX
            start_rc = mosaic_grid.mosaic_px_to_coarse_grid(*start_px)
            goal_rc = mosaic_grid.mosaic_px_to_coarse_grid(*goal_px)

        if cost_grid is None:
            return {"path_grid": [], "path_mosaic": [], "distance_cm": 0,
                    "cost": 0, "cell_px": cell_px, "success": False}

        rows, cols = cost_grid.shape

        # Clamp to grid bounds
        start_rc = (max(0, min(rows - 1, start_rc[0])),
                    max(0, min(cols - 1, start_rc[1])))
        goal_rc = (max(0, min(rows - 1, goal_rc[0])),
                   max(0, min(cols - 1, goal_rc[1])))

        path = self._astar(cost_grid, start_rc, goal_rc)

        if path is None:
            return {"path_grid": [], "path_mosaic": [], "distance_cm": 0,
                    "cost": 0, "cell_px": cell_px, "success": False}

        # Convert grid path to mosaic pixel coords
        path_mosaic = []
        for r, c in path:
            cx = c * cell_px + cell_px // 2
            cy = r * cell_px + cell_px // 2
            path_mosaic.append((cx, cy))

        # Physical distance
        cell_cm = config.GRID_CELL_SIZE_CM * (cell_px / config.MOSAIC_GRID_CELL_PX)
        distance_cm = self._path_length(path) * cell_cm

        # Total cost
        total_cost = sum(float(cost_grid[r, c]) for r, c in path)

        return {
            "path_grid": path,
            "path_mosaic": path_mosaic,
            "distance_cm": round(distance_cm, 2),
            "cost": round(total_cost, 2),
            "cell_px": cell_px,
            "success": True,
        }

    def visualize_route(self, mosaic_grid, route_result, mosaic_image=None,
                        output_path=None):
        """Draw the planned route on the cost grid or mosaic image.

        Args:
            mosaic_grid: MosaicGrid instance.
            route_result: dict from plan_route().
            mosaic_image: Optional np.ndarray — if provided, draws route on mosaic.
            output_path: Optional path to save the visualization.

        Returns:
            np.ndarray (BGR) visualization image.
        """
        cell_px = route_result["cell_px"]

        if config.SEG_ENABLED:
            cost_grid = mosaic_grid.get_fine_cost_grid()
            rows, cols = mosaic_grid.fine_rows, mosaic_grid.fine_cols
        else:
            cost_grid = mosaic_grid.get_coarse_cost_grid()
            rows, cols = mosaic_grid.coarse_rows, mosaic_grid.coarse_cols

        if cost_grid is None:
            return np.zeros((100, 100, 3), dtype=np.uint8)

        # Dynamic cell visualization size — scale down for large grids
        cell_vis = max(4, min(64, 512 // max(rows, cols)))

        if mosaic_image is not None:
            # Draw route directly on mosaic
            vis = mosaic_image.copy()
            path_mosaic = route_result.get("path_mosaic", [])
            for i in range(len(path_mosaic) - 1):
                pt1 = path_mosaic[i]
                pt2 = path_mosaic[i + 1]
                cv2.line(vis, pt1, pt2, (0, 255, 0), 2)
            if path_mosaic:
                cv2.circle(vis, path_mosaic[0], 6, (0, 255, 255), -1)   # start
                cv2.circle(vis, path_mosaic[-1], 6, (255, 0, 255), -1)  # goal
        else:
            # Draw grid-based visualization
            vis_h = rows * cell_vis
            vis_w = cols * cell_vis
            vis = np.zeros((vis_h, vis_w, 3), dtype=np.uint8)

            # Color cells by cost
            for r in range(rows):
                for c in range(cols):
                    cost = float(cost_grid[r, c])
                    if cost >= _IMPASSABLE_COST:
                        color = (0, 0, 180)  # red — impassable
                    elif cost > 10:
                        color = (0, 140, 255)  # orange — hazardous
                    elif cost > 1:
                        color = (0, 200, 255)  # yellow — caution
                    else:
                        color = (180, 200, 180)  # green — safe
                    y0 = r * cell_vis
                    x0 = c * cell_vis
                    vis[y0:y0 + cell_vis, x0:x0 + cell_vis] = color

            # Draw route
            path_grid = route_result.get("path_grid", [])
            for i in range(len(path_grid) - 1):
                r1, c1 = path_grid[i]
                r2, c2 = path_grid[i + 1]
                pt1 = (c1 * cell_vis + cell_vis // 2, r1 * cell_vis + cell_vis // 2)
                pt2 = (c2 * cell_vis + cell_vis // 2, r2 * cell_vis + cell_vis // 2)
                cv2.line(vis, pt1, pt2, (0, 255, 0), max(1, cell_vis // 4))
            if path_grid:
                sr, sc = path_grid[0]
                er, ec = path_grid[-1]
                cv2.circle(vis, (sc * cell_vis + cell_vis // 2, sr * cell_vis + cell_vis // 2),
                           cell_vis // 2, (0, 255, 255), -1)
                cv2.circle(vis, (ec * cell_vis + cell_vis // 2, er * cell_vis + cell_vis // 2),
                           cell_vis // 2, (255, 0, 255), -1)

        if output_path:
            os.makedirs(os.path.dirname(output_path), exist_ok=True)
            cv2.imwrite(output_path, vis)

        return vis

    # ------------------------------------------------------------------
    # A* implementation
    # ------------------------------------------------------------------

    def _astar(self, cost_grid, start, goal):
        """A* search on a 2D cost grid with 8-connected neighbors.

        Args:
            cost_grid: np.ndarray (rows, cols) float32 — traversal cost per cell.
            start: (row, col) tuple.
            goal: (row, col) tuple.

        Returns:
            List of (row, col) from start to goal, or None if unreachable.
        """
        rows, cols = cost_grid.shape

        # Priority queue: (f_score, counter, (row, col))
        counter = 0
        open_set = [(0, counter, start)]
        came_from = {}
        g_score = {start: 0.0}

        # 8-directional neighbors with diagonal cost sqrt(2)
        neighbors = [
            (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414),
        ]

        while open_set:
            _, _, current = heapq.heappop(open_set)

            if current == goal:
                return self._reconstruct_path(came_from, current)

            for dr, dc, step_cost in neighbors:
                nr, nc = current[0] + dr, current[1] + dc
                if nr < 0 or nr >= rows or nc < 0 or nc >= cols:
                    continue

                cell_cost = float(cost_grid[nr, nc])
                if cell_cost >= _IMPASSABLE_COST:
                    continue

                tentative_g = g_score[current] + step_cost * cell_cost

                neighbor = (nr, nc)
                if tentative_g < g_score.get(neighbor, float("inf")):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + self._heuristic(neighbor, goal)
                    counter += 1
                    heapq.heappush(open_set, (f, counter, neighbor))

        return None  # No path found

    @staticmethod
    def _heuristic(a, b):
        """Octile distance heuristic for 8-connected grid."""
        dr = abs(a[0] - b[0])
        dc = abs(a[1] - b[1])
        return max(dr, dc) + 0.414 * min(dr, dc)

    @staticmethod
    def _reconstruct_path(came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    @staticmethod
    def _path_length(path):
        """Sum of Euclidean step distances in grid-cell units."""
        total = 0.0
        for i in range(len(path) - 1):
            dr = path[i + 1][0] - path[i][0]
            dc = path[i + 1][1] - path[i][1]
            total += math.sqrt(dr * dr + dc * dc)
        return total
