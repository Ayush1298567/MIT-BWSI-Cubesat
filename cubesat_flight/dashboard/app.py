# dashboard/app.py — Lightweight Flask dashboard for mosaic + segmentation
#
# Endpoints:
#   GET /                        — Dashboard HTML page
#   GET /api/cost_grid           — JSON cost grid data (coarse + fine)
#   GET /api/segmentation_overlay — PNG segmentation overlay (BGRA)
#   GET /api/route               — Plan a route (query params: x0,y0,x1,y1)
#   GET /api/status              — Pipeline status summary

import io
import json

import cv2
import numpy as np

try:
    from flask import Flask, Response, jsonify, request, send_file
except ImportError:
    Flask = None

import config


def create_app(pipeline):
    """Create and configure the Flask dashboard app.

    Args:
        pipeline: ProcessingPipeline instance (shared with flight software).

    Returns:
        Flask app instance, or None if Flask is not installed.
    """
    if Flask is None:
        return None

    app = Flask(__name__)

    @app.route("/")
    def index():
        return Response(_DASHBOARD_HTML, content_type="text/html")

    @app.route("/api/cost_grid")
    def cost_grid():
        mg = pipeline.mosaic_grid

        result = {
            "mosaic_w": mg.mosaic_w,
            "mosaic_h": mg.mosaic_h,
            "coarse_cell_px": config.MOSAIC_GRID_CELL_PX,
            "coarse_rows": mg.coarse_rows,
            "coarse_cols": mg.coarse_cols,
            "coarse_grid": None,
        }

        cg = mg.get_coarse_cost_grid()
        if cg is not None:
            result["coarse_grid"] = cg.tolist()

        if config.SEG_ENABLED:
            result["fine_cell_px"] = config.SEG_GRID_CELL_PX
            result["fine_rows"] = mg.fine_rows
            result["fine_cols"] = mg.fine_cols
            fg = mg.get_fine_cost_grid()
            if fg is not None:
                result["fine_grid"] = fg.tolist()

        return jsonify(result)

    @app.route("/api/segmentation_overlay")
    def segmentation_overlay():
        overlay = pipeline.get_segmentation_overlay()
        if overlay is None:
            return Response("No segmentation data", status=404)

        success, buf = cv2.imencode(".png", overlay)
        if not success:
            return Response("Encoding failed", status=500)

        return send_file(
            io.BytesIO(buf.tobytes()),
            mimetype="image/png",
            download_name="segmentation_overlay.png",
        )

    @app.route("/api/route")
    def plan_route():
        try:
            x0 = int(request.args["x0"])
            y0 = int(request.args["y0"])
            x1 = int(request.args["x1"])
            y1 = int(request.args["y1"])
        except (KeyError, ValueError):
            return jsonify({"error": "Required params: x0, y0, x1, y1"}), 400

        route = pipeline.plan_route((x0, y0), (x1, y1))

        # Convert numpy types for JSON serialization
        serializable = {
            "success": route["success"],
            "distance_cm": route["distance_cm"],
            "cost": route["cost"],
            "cell_px": route["cell_px"],
            "path_mosaic": route.get("path_mosaic", []),
            "num_steps": len(route.get("path_grid", [])),
        }
        return jsonify(serializable)

    @app.route("/api/status")
    def status():
        mg = pipeline.mosaic_grid
        last = pipeline.last_route

        return jsonify({
            "seg_enabled": config.SEG_ENABLED,
            "mosaic_w": mg.mosaic_w,
            "mosaic_h": mg.mosaic_h,
            "fine_grid_size": [mg.fine_rows, mg.fine_cols] if config.SEG_ENABLED else None,
            "coarse_grid_size": [mg.coarse_rows, mg.coarse_cols],
            "last_route_success": last["success"] if last else None,
            "last_route_distance_cm": last["distance_cm"] if last else None,
        })

    return app


# Minimal embedded dashboard HTML
_DASHBOARD_HTML = """\
<!DOCTYPE html>
<html>
<head>
<title>CubeSat Segmentation Dashboard</title>
<style>
  body { font-family: monospace; background: #1a1a2e; color: #eee; margin: 20px; }
  h1 { color: #e94560; }
  .panel { background: #16213e; padding: 15px; margin: 10px 0; border-radius: 8px; }
  button { background: #e94560; color: white; border: none; padding: 8px 16px;
           border-radius: 4px; cursor: pointer; margin: 4px; }
  button:hover { background: #c73e54; }
  #overlay { max-width: 100%; border: 1px solid #444; }
  #status, #route-result { white-space: pre; font-size: 13px; }
  .grid-info { display: flex; gap: 20px; flex-wrap: wrap; }
  .grid-info div { flex: 1; min-width: 200px; }
  input { background: #0f3460; color: #eee; border: 1px solid #444;
          padding: 4px 8px; width: 60px; border-radius: 4px; }
</style>
</head>
<body>
<h1>MuraltZ Segmentation Dashboard</h1>

<div class="panel">
  <h3>Status</h3>
  <div id="status">Loading...</div>
  <button onclick="refreshStatus()">Refresh</button>
</div>

<div class="panel">
  <h3>Segmentation Overlay</h3>
  <button onclick="loadOverlay()">Load Overlay</button>
  <br><br>
  <img id="overlay" alt="No overlay loaded">
</div>

<div class="panel">
  <h3>Route Planner</h3>
  <label>Start: x=<input id="x0" value="0"> y=<input id="y0" value="0"></label>
  <label>Goal: x=<input id="x1" value="100"> y=<input id="y1" value="100"></label>
  <button onclick="planRoute()">Plan Route</button>
  <div id="route-result"></div>
</div>

<script>
async function refreshStatus() {
  const r = await fetch('/api/status');
  document.getElementById('status').textContent = JSON.stringify(await r.json(), null, 2);
}
async function loadOverlay() {
  document.getElementById('overlay').src = '/api/segmentation_overlay?' + Date.now();
}
async function planRoute() {
  const p = ['x0','y0','x1','y1'].map(id => id+'='+document.getElementById(id).value).join('&');
  const r = await fetch('/api/route?' + p);
  document.getElementById('route-result').textContent = JSON.stringify(await r.json(), null, 2);
}
refreshStatus();
</script>
</body>
</html>
"""
