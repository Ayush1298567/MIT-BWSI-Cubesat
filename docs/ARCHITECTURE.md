# MuraltZ CubeSat — Flight Software Architecture v3
## Raspberry Pi 4 (2GB) | Artemis Lunar Navigator | MIT BWSI 2025-2026
## Fixes applied: #1 grid cell input, #2 physical flow + loose thresholds, #3 blur calibration, #4 data budget math, #5 P2 aging, #7 auto-exposure, #10 nadir hysteresis, #11 GCS offline handling

---

## 1. Hardware

| Component | Model | Interface | Software usage |
|-----------|-------|-----------|----------------|
| Flight Computer | Raspberry Pi 4 (2GB) | — | Hostname: `cubesat`, User: `cubesat` |
| IMU | LSM6DSO32 | I2C (addr 0x6A) | Stability + nadir gating for captures |
| Camera | Pi Camera Module 3 (IMX708) | CSI ribbon | `from picamera2 import Picamera2`. NOT libcamera. |
| Battery | USB Power Bank | USB | Power draw measured manually with USB power meter |
| WiFi | Built-in Pi 4 | 2.4 GHz | Pushes files to GCS + receives commands |
| CPU Temp | sysfs thermal sensor | `/sys/class/thermal/thermal_zone0/temp` | Monitored every 10 sec |

---

## 2. Directory Structure

```
/home/cubesat/cubesat_flight/
├── main.py                    # State machine loop
├── config.py                  # All parameters
├── protocol.py                # Shared contract with GCS (ports, schemas, ACK bytes)
├── states/
│   ├── __init__.py
│   ├── boot.py                # Self-test
│   ├── imaging.py             # IMU-gated capture loop
│   ├── idle.py                # Queue building, aging, cleanup, ground commands
│   ├── downlink.py            # Throttled TCP push
│   └── safe_mode.py           # Error recovery
├── sensors/
│   ├── __init__.py
│   ├── imu.py                 # LSM6DSO32 I2C wrapper
│   └── camera.py              # picamera2 wrapper
├── processing/
│   ├── __init__.py
│   ├── quality.py             # Blur + exposure + motion scoring
│   ├── coverage.py            # 8x8 grid tracking
│   └── metadata.py            # _meta.json sidecar generator
├── comms/
│   ├── __init__.py
│   ├── packet.py              # Header builder
│   ├── transfer.py            # TCP client, throttled, ACK/NACK
│   └── command_listener.py    # Background thread — receives GCS commands
├── storage/
│   ├── __init__.py
│   └── manager.py             # Queue persistence, image index, capacity, aging
└── utils/
    ├── __init__.py
    ├── watchdog.py             # Daemon thread restart
    ├── logger.py               # NOT logging.py (avoids stdlib clash)
    ├── telemetry.py            # Real sensor data packager
    └── thermal.py              # CPU temp monitoring
```

---

## 3. protocol.py — Shared Interface Contract

Identical copy in both CubeSat and GCS codebases.

```python
DATA_PORT = 5000
COMMAND_PORT = 5001

# Transfer header: JSON + newline, then raw bytes
# {"type": "image"|"telemetry", "filename": "...", "file_size": N, "md5": "...", "metadata": {...}}

ACK = b'\x06'
NACK = b'\x15'

# Commands (GCS → CubeSat): JSON + newline
# {"cmd": "retransmit", "image_id": "..."}
# {"cmd": "priority_cell", "row": R, "col": C}
# {"cmd": "set_cell", "row": R, "col": C}       ← operator grid cell override
# {"cmd": "adjust_exposure", "exposure_us": N}
# {"cmd": "enter_safe_mode"}
# {"cmd": "resume_normal"}
# {"cmd": "status_request"}

# File naming: pass{N}_img{MM}_{YYYYMMDD_HHMMSS}.jpg + _meta.json
```

---

## 4. config.py

```python
MISSION_NAME = "MuraltZ"
CUBESAT_ID = "MURALTZ-01"

# === STATE TIMING ===
BOOT_TIMEOUT = 30
IMAGING_WINDOW_SEC = 75
IDLE_DURATION_SEC = 30
DOWNLINK_WINDOW_SEC = 60
PASS_CYCLE_DELAY_SEC = 10

# === CAMERA ===
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
JPEG_QUALITY = 70
JPEG_MAX_SIZE_BYTES = 40000          # If > this at Q=70, recompress at Q=60
CAPTURE_INTERVAL_SEC = 3.0
MAX_IMAGES_PER_PASS = 20

# [FIX #7] Camera runs in AUTO-EXPOSURE mode by default.
# We do NOT manually calibrate exposure at pass start because the flashlight
# creates uneven brightness across the surface. Auto-exposure lets the camera
# continuously adapt. We log the auto-chosen values from get_metadata().
CAMERA_USE_AUTO_EXPOSURE = True

# === CAMERA CALIBRATION (MEASURED) ===
CAMERA_HEIGHT_CM = 0.0               # Fill in: measured lens-to-surface distance
CAMERA_FOV_H_DEG = 62.2
CAMERA_FOV_V_DEG = 48.8

# === IMU THRESHOLDS ===
# [FIX #2] Loosened for handheld operation. A real satellite at 100km would use
# much tighter tolerances, but handheld demo has more noise.
ANGULAR_RATE_THRESHOLD = 1.0         # rad/s (was 0.5 — too tight for handheld)
NADIR_TOLERANCE_DEG = 45             # degrees (was 30 — too tight for handheld)
# [FIX #10] Hysteresis: once nadir is confirmed at < 45°, stay confirmed
# until angle exceeds NADIR_EXIT_DEG. Prevents rapid capture-skip toggling.
NADIR_EXIT_DEG = 55                  # Must exceed this to lose nadir lock

# === QUALITY GATES ===
# [FIX #3] Blur threshold MUST be calibrated against your actual surface.
# Before demo: take 10 sharp + 10 blurry photos of the real surface.
# Set threshold halfway between lowest sharp variance and highest blurry variance.
# Default of 50 is conservative — will likely need adjustment.
BLUR_THRESHOLD = 50.0                # ← CALIBRATE THIS. Default is a guess.
EXPOSURE_MIN = 15
EXPOSURE_MAX = 240
QUALITY_WEIGHT_BLUR = 0.3
QUALITY_WEIGHT_EXPOSURE = 0.25
QUALITY_WEIGHT_MOTION = 0.2
QUALITY_WEIGHT_NOVELTY = 0.25

# === COVERAGE GRID ===
GRID_ROWS = 8
GRID_COLS = 8
# [FIX #1] NO TRAJECTORY_MAP. Grid cell is set by operator input, not elapsed time.
# Operator types "cell 2 3" into the CubeSat terminal when they move to a new area.
# All subsequent captures are tagged with that cell until updated.
# This is reliable — no desync from timing drift.
DEFAULT_GRID_CELL = (0, 0)

# === DOWNLINK ===
GROUND_STATION_IP = "192.168.1.100"
DATA_PORT = 5000
COMMAND_PORT = 5001
THROTTLE_BYTES_PER_SEC = 1200        # 9600 bps / 8 = flight UHF equivalent
ACK_TIMEOUT_SEC = 5
MAX_RETRIES_PER_IMAGE = 3

# [FIX #4] Data budget computed from window × rate. Not a separate constant.
# At 1200 B/s × 60 sec = 72,000 bytes ≈ 72 KB ≈ 2-3 images per window.
# This is the REAL constraint — not an arbitrary 2 MB cap.
# DATA_BUDGET = THROTTLE_BYTES_PER_SEC * DOWNLINK_WINDOW_SEC (computed at runtime)

# [FIX #11] Max consecutive failed GCS connections before giving up
MAX_GCS_CONNECT_FAILURES = 3         # After this, stop trying until manual command

# === PRIORITY TIERS ===
# [FIX #5] P2 aging lowered from 5 to 2 so it actually triggers in a 3-pass demo
P2_AGING_PASSES = 2                  # Was 5 — never triggered in 3-pass demo

# === STORAGE ===
IMAGE_DIR = "/home/cubesat/cubesat_flight/data/images"
TELEMETRY_DIR = "/home/cubesat/cubesat_flight/data/telemetry"
LOG_DIR = "/home/cubesat/cubesat_flight/data/logs"
QUEUE_FILE = "/home/cubesat/cubesat_flight/data/queue.json"
IMAGE_INDEX_FILE = "/home/cubesat/cubesat_flight/data/image_index.json"
RECOVERY_FILE = "/home/cubesat/cubesat_flight/data/recovery.json"
STORAGE_WARNING_PCT = 80
STORAGE_CRITICAL_PCT = 98

# === THERMAL ===
CPU_TEMP_WARNING_C = 70
CPU_TEMP_CRITICAL_C = 80
THERMAL_CHECK_INTERVAL_SEC = 10

# === WATCHDOG ===
WATCHDOG_TIMEOUT_SEC = 30
WATCHDOG_CHECK_INTERVAL_SEC = 5

# === MEASURED POWER (fill in from USB power meter) ===
MEASURED_IDLE_POWER_W = 0.0
MEASURED_IMAGING_POWER_W = 0.0
MEASURED_PROCESSING_POWER_W = 0.0
MEASURED_DOWNLINK_POWER_W = 0.0

# === DEMO SCALE ===
DEMO_GSD_MM_PER_PX = 0.12
FLIGHT_GSD_M_PER_PX = 29.5
SCALE_FACTOR = 250_000
```

---

## 5. Physical Demo Flow

**[FIX #2] The operator MUST understand this flow. The IMU gates captures — the CubeSat only shoots when held still and pointing down.**

```
1. Operator holds CubeSat at ~40 cm above surface
2. Operator types: "cell 0 0" (sets current grid cell)            ← [FIX #1]
3. Operator moves CubeSat over cell (0,0) area of the surface
4. Operator STOPS moving and holds steady for 2-3 seconds
5. Software detects: angular_rate < 1.0 rad/s AND nadir angle < 45°
6. Software logs: "STABLE + NADIR — capturing"
7. Camera fires — real photo of real surface
8. Operator sees "Captured pass1_img00" in terminal
9. Operator moves to next area, types "cell 0 1"
10. Repeat: stop → stabilize → capture → move

Between passes:
  Operator types "end_pass" → PROCESSING → IDLE → DOWNLINK happens automatically
  Operator modifies the surface (move a rock, add a crater) for change detection
  Operator types "start_pass" to begin next pass
```

**Key point:** The CubeSat does NOT capture during movement. It captures during stable holds. This is correct satellite behavior — a tumbling satellite can't take sharp photos. The demo naturally shows this: move, stop, capture, move, stop, capture.

---

## 6. State Machine

```
BOOT → IMAGING → IDLE → DOWNLINK → [increment pass] → IMAGING → ...
          ↘                                               ↗
           → (error) → SAFE_MODE → (recovery) →
```

### BOOT
- Test IMU: read real accel, confirm Z ≈ 9.8 ± 1.0 m/s²
- Test camera: take test photo, confirm > 10 KB and decodable
- Test storage: check capacity via `shutil.disk_usage()`
- Ping GCS: attempt ping. If fails, log "GCS unreachable — autonomous mode" but do NOT fail boot
- Read CPU temp, log it
- Check for recovery file → resume from last pass if exists
- If camera OR IMU fail → safe mode
- If storage > 98% → downlink-only mode

### IMAGING
- Duration: `IMAGING_WINDOW_SEC` (75 sec) or until `end_pass` command
- **[FIX #1] Grid cell set by operator input.** Software tracks `current_grid_cell` variable. Operator updates it by typing `cell R C` in the terminal. All captures tagged with current cell until changed.
- **[FIX #7] Camera runs in auto-exposure.** No manual calibration. Auto-exposure continuously adapts to the flashlight's uneven brightness. Each capture's real auto-chosen exposure is read from `camera.get_metadata()` and logged in the sidecar.
- **[FIX #10] IMU nadir check uses hysteresis.** Once angle drops below `NADIR_TOLERANCE_DEG` (45°), set `nadir_locked = True`. Stay locked until angle exceeds `NADIR_EXIT_DEG` (55°). This prevents rapid toggling when hovering near the threshold. Stability check (`angular_rate < 1.0 rad/s`) has no hysteresis — it's either stable or not.
- Loop every `CAPTURE_INTERVAL_SEC`:
  1. Check storage capacity — if critical, stop
  2. Check CPU temp — if warning, double interval; if critical, safe mode
  3. Read IMU: `is_stable()` AND `nadir_locked`
  4. If both: capture, read angular rate + camera metadata at capture time, run quality gate, generate _meta.json, assign grid cell from `current_grid_cell`
  5. If not: log skip with reason, continue
  6. **[FIX #3] Quality gate blur threshold is calibrated.** The default (50) is a conservative starting point. MUST be adjusted based on test photos of the actual surface before demo day.
- Adaptive JPEG: if image > 40 KB at Q=70, recompress at Q=60. Log it.

### IDLE
- Duration: `IDLE_DURATION_SEC` (30 sec)
- Build priority queue sorted by: tier (P1 > P2 > P3), then score descending
- **[FIX #5]** Age images: P2 images from passes older than `P2_AGING_PASSES` (2) → downgrade to P3. With 3 passes, this fires during pass 3's IDLE for pass 1's P2 images. Log: "Pass 1 P2 images aged to P3."
- If storage > 80%: delete P3 images oldest first
- Process pending ground commands
- Save queue + index + coverage to disk

### DOWNLINK
- Duration: up to `DOWNLINK_WINDOW_SEC` (60 sec)
- **[FIX #4] Data budget = `THROTTLE_BYTES_PER_SEC * DOWNLINK_WINDOW_SEC`** = 1200 × 60 = 72,000 bytes. This is the real constraint. At ~28 KB per image, that's ~2 images per window. This is realistic and the priority queue makes it meaningful — you're choosing which 2 of 8 images to send.
- Send telemetry packet first (small, ~500 bytes)
- Pop images from queue highest-priority first:
  1. Check if `bytes_sent + file_size > data_budget` → stop if exceeded
  2. Send via throttled TCP (1200 B/s chunks with real 1-sec sleep)
  3. Wait for ACK/NACK
  4. ACK → mark sent, remove from queue
  5. NACK → retry up to `MAX_RETRIES_PER_IMAGE`, then skip as corrupt
  6. Socket error → log, close socket, image stays queued, stop downlink
- **[FIX #11] GCS offline handling:** Track consecutive connection failures. If `connect()` fails `MAX_GCS_CONNECT_FAILURES` (3) times in a row across passes, stop attempting downlink. Log: "GCS unreachable after 3 attempts — suspending downlink. Images queuing locally." Resume when: (a) a ground command arrives on the command port (meaning GCS is back), or (b) operator manually sends `retry_downlink`. Don't burn the whole window on timeouts.

### SAFE MODE
- Entered on: hardware failure, CPU temp critical, unrecoverable error, ground command
- Stop camera, close sockets
- Command listener still runs, thermal still monitors
- Exit via: ground command `resume_normal`, or watchdog restart

---

## 7. Module Specifications

### 7.1 `sensors/imu.py`

Reads physical LSM6DSO32 over I2C every call. No caching.

| Method | Returns | Notes |
|--------|---------|-------|
| `get_acceleration()` | `(x, y, z)` m/s² | At rest pointing down: `(~0, ~0, ~-9.8)` |
| `get_gyro()` | `(x, y, z)` rad/s | At rest: `(~0, ~0, ~0)` |
| `get_angular_rate()` | float | `sqrt(gx² + gy² + gz²)` |
| `is_stable()` | bool | `angular_rate < ANGULAR_RATE_THRESHOLD` (1.0 rad/s) |
| `get_nadir_angle()` | float degrees | Angle between accel vector and -Z axis |
| `get_orientation()` | dict | `{roll, pitch, yaw, accel_mag}` for metadata |

**[FIX #10] Nadir hysteresis is NOT in the IMU module — it's in the imaging state.** The IMU just reports the raw angle. The imaging loop maintains the `nadir_locked` flag:

```
if not nadir_locked:
    if imu.get_nadir_angle() < NADIR_TOLERANCE_DEG:
        nadir_locked = True
else:
    if imu.get_nadir_angle() > NADIR_EXIT_DEG:
        nadir_locked = False
```

### 7.2 `sensors/camera.py`

Controls physical Pi Camera Module 3 via picamera2.

| Method | What it does |
|--------|-------------|
| `__init__(w, h)` | Still config, `start()`, sleep 2 sec warm-up |
| `capture(filepath)` | Real photo → real JPEG on disk |
| `capture_with_recompress(filepath, max_bytes)` | Capture at Q=70. If > max_bytes, redo at Q=60. |
| `set_low_light_mode()` | Exposure 100ms + Gain 8.0 for eclipse demo |
| `set_normal_mode()` | Back to auto-exposure |
| `get_metadata()` | Real dict: ExposureTime, AnalogueGain, Lux, etc. |
| `close()` | Clean shutdown |

**[FIX #7]** Camera starts in auto-exposure mode and stays there. No `calibrate_exposure()` method. The flashlight creates uneven brightness — auto-exposure handles this continuously. We log the auto-chosen settings from `get_metadata()` in every image's sidecar.

### 7.3 `processing/quality.py`

| Check | Method | Threshold | Notes |
|-------|--------|-----------|-------|
| Blur | `cv2.Laplacian(gray, cv2.CV_64F).var()` | `BLUR_THRESHOLD` (default 50) | **[FIX #3]** MUST calibrate against real surface. Sand has low variance even when sharp. Take 10 sharp + 10 blurry test photos, set threshold halfway. |
| Exposure | `numpy.mean(gray)` | 15 – 240 | Standard |
| Motion blur | `angular_rate` from IMU at capture time | `ANGULAR_RATE_THRESHOLD` (1.0 rad/s) | Ties ADCS to imaging quality |
| Novelty | From coverage tracker | 1.0 new / 0.5 better / 0.1 redundant | Drives priority queue |

Combined: `Q = 0.3*blur + 0.25*exp + 0.2*motion + 0.25*novelty`. Any hard fail → Q=0 → rejected.

### 7.4 `processing/coverage.py`

8×8 grid matching taped surface.

**[FIX #1]** Grid cell is NOT assigned by elapsed time. It's set by operator input:
- Operator types `cell R C` in terminal
- Software sets `current_grid_cell = (R, C)`
- All subsequent captures tagged with that cell until operator updates it
- If operator forgets to update, images get tagged to the same cell. Novelty scoring handles this gracefully — redundant captures get low novelty, so they sort to the bottom of the queue.

Each cell stores: covered, best_quality_score, best_image_filename, pass_number, timestamp.

### 7.5 `processing/metadata.py`

Generates `_meta.json` sidecar for every captured image:

```json
{
    "filename": "pass3_img07_20260315_144500.jpg",
    "pass_number": 3,
    "image_sequence": 7,
    "timestamp": "2026-03-15T14:45:00Z",
    "grid_cell": [2, 3],
    "imu": {
        "roll": 1.2, "pitch": -0.8, "yaw": 45.3,
        "accel_magnitude": 0.12, "angular_rate": 0.03,
        "stable": true, "nadir_locked": true, "nadir_angle_deg": 12.4
    },
    "camera": {
        "exposure_us": 10000, "analog_gain": 1.5,
        "jpeg_quality": 70, "recompressed": false
    },
    "quality": {
        "blur_variance": 245.7, "blur_score": 0.72,
        "exposure_mean": 118, "exposure_score": 0.85,
        "motion_score": 0.94, "novelty_score": 1.0,
        "combined_score": 0.87, "passed_gate": true,
        "rejection_reason": null
    },
    "priority_tier": "P1",
    "file_size_bytes": 28400,
    "md5": "a1b2c3d4e5f6...",
    "downlink_status": "pending"
}
```

### 7.6 `comms/transfer.py`

Throttled TCP client. Sends to `GROUND_STATION_IP:DATA_PORT`.

Protocol: JSON header (newline-terminated) → raw bytes in 1200-byte chunks with real 1-sec sleep → wait for ACK/NACK byte.

**[FIX #4]** Data budget computed at runtime:
```python
data_budget = THROTTLE_BYTES_PER_SEC * DOWNLINK_WINDOW_SEC  # = 72,000 bytes
```
At ~28 KB per image, this is ~2 images per 60-second window. The "data budget exhausted" log will actually fire.

**[FIX #11]** Connection failure tracking:
```python
consecutive_failures = 0

def connect():
    try:
        socket.connect((IP, PORT))
        consecutive_failures = 0
    except:
        consecutive_failures += 1
        if consecutive_failures >= MAX_GCS_CONNECT_FAILURES:
            raise GCSUnreachableError("Suspending downlink after 3 failures")
```

### 7.7 `comms/command_listener.py`

Background daemon thread on `COMMAND_PORT`. Accepts JSON commands from GCS.

| Command | Effect |
|---------|--------|
| `retransmit` | Move image to top of queue |
| `priority_cell` | Boost novelty for that cell |
| `set_cell` | Override `current_grid_cell` (backup for operator input) |
| `adjust_exposure` | Set camera exposure for next captures |
| `enter_safe_mode` | Immediate safe mode |
| `resume_normal` | Exit safe mode → IDLE |
| `status_request` | Send telemetry packet immediately |
| `retry_downlink` | Reset `consecutive_failures` counter, resume downlink attempts |

### 7.8 `storage/manager.py`

| Method | What |
|--------|------|
| `check_capacity()` | `shutil.disk_usage()` → percent used |
| `save_queue(q)` / `load_queue()` | JSON persistence (survives reboot) |
| `save_image_index()` / `load_image_index()` | Full tracking of every image |
| `update_image_index(filename, meta)` | Add or update one image entry |
| `mark_sent(filename)` | status → "sent" |
| `mark_failed(filename)` | status → "failed", increment retry count |
| `build_priority_queue(current_pass)` | Sort undownlinked by tier then score |
| `age_images(current_pass)` | **[FIX #5]** P2 older than 2 passes → P3 |
| `cleanup_p3()` | Delete oldest P3 images until storage < 80% |
| `save_recovery_state(d)` / `load_recovery_state()` | For watchdog |

### 7.9 `utils/thermal.py`

Reads `/sys/class/thermal/thermal_zone0/temp` every 10 sec in a background thread.

- `get_cpu_temp()` → real °C
- `is_warning()` → True if > 70°C
- `is_critical()` → True if > 80°C

At warning: imaging loop doubles capture interval. At critical: force safe mode.

### 7.10 `utils/watchdog.py`

Daemon thread. Main loop calls `pet()` every iteration. If no pet within 30 sec → save state to recovery file → `os.execv()` restart. On restart, `main.py` loads recovery file and resumes from last pass.

### 7.11 `utils/logger.py`

Format: `{ISO_timestamp} [{state}] [{level}] {message}`

NOT named `logging.py`. Appends to rotating file in `LOG_DIR`. Last 50 entries kept in memory for telemetry.

### 7.12 `utils/telemetry.py`

All values from real sources:

```json
{
    "type": "telemetry",
    "timestamp": "...", "cubesat_id": "MURALTZ-01",
    "pass_number": 3, "state": "DOWNLINK", "uptime_sec": 245,
    "imu": {"accel": [...], "gyro": [...], "angular_rate": 0.004,
            "stable": true, "nadir_locked": true, "nadir_angle_deg": 12.4},
    "camera": {"exposure_us": 10000, "analog_gain": 1.5, "mode": "auto"},
    "thermal": {"cpu_temp_c": 58.3, "throttled": false},
    "storage": {"used_pct": 12.4, "free_mb": 3500},
    "imaging": {"captured_this_pass": 8, "captured_total": 24,
                "rejected_total": 6, "rejection_breakdown": {"blur": 3, "underexposed": 2, "motion_blur": 1}},
    "downlink": {"queued": 5, "sent_total": 15, "bytes_this_pass": 56000,
                 "budget_remaining": 16000, "gcs_reachable": true},
    "coverage": {"cells_filled": 40, "cells_total": 64, "pct": 62.5},
    "errors": [], "recent_log": ["..."]
}
```

---

## 8. main.py — Logic (Pseudocode)

```
main():
    thermal.start_monitoring()
    watchdog.start()
    storage = StorageManager()
    
    recovery = storage.load_recovery_state()
    pass_number = recovery["pass_number"] if recovery else 0
    
    # BOOT
    imu = IMU()
    assert 8.0 < abs(imu.get_acceleration()[2]) < 11.0
    camera = Camera(IMAGE_WIDTH, IMAGE_HEIGHT)  # auto-exposure by default
    assert os.path.getsize(camera.capture("/tmp/test.jpg")) > 10000
    gs_reachable = ping(GROUND_STATION_IP)
    log(f"GCS: {'reachable' if gs_reachable else 'UNREACHABLE — autonomous mode'}")
    
    coverage = Coverage(); coverage.load()
    command_listener = CommandListener(); command_listener.start()
    transfer = Transfer()
    quality_gate = QualityGate()
    
    current_grid_cell = DEFAULT_GRID_CELL
    nadir_locked = False
    gcs_suspended = False   # [FIX #11]
    
    while True:
        watchdog.pet()
        pass_number += 1
        
        # === IMAGING ===
        log(f"=== PASS {pass_number} — IMAGING ===")
        images_this_pass = []
        rejections = {"blur": 0, "underexposed": 0, "overexposed": 0, "motion_blur": 0}
        img_start = time.time()
        nadir_locked = False    # Reset each pass
        
        while (time.time() - img_start) < IMAGING_WINDOW_SEC:
            watchdog.pet()
            
            # Check for commands (cell update, end_pass, etc.)
            for cmd in command_listener.get_pending():
                if cmd["cmd"] == "set_cell":
                    current_grid_cell = (cmd["row"], cmd["col"])
                    log(f"Grid cell → {current_grid_cell}")
                elif cmd["cmd"] == "end_pass":
                    break  # Exit imaging loop
                else:
                    handle_command(cmd, ...)
            
            # Also check stdin for operator "cell R C" or "end_pass"
            operator_input = check_stdin_nonblocking()
            if operator_input:
                if operator_input.startswith("cell"):
                    parts = operator_input.split()
                    current_grid_cell = (int(parts[1]), int(parts[2]))
                    log(f"Grid cell → {current_grid_cell}")
                elif operator_input == "end_pass":
                    break
            
            if len(images_this_pass) >= MAX_IMAGES_PER_PASS:
                break
            if storage.is_critical():
                break
            if thermal.is_critical():
                enter_safe_mode(); break
            
            interval = CAPTURE_INTERVAL_SEC
            if thermal.is_warning():
                interval *= 2
            
            # IMU gating with hysteresis [FIX #10]
            nadir_angle = imu.get_nadir_angle()
            if not nadir_locked:
                if nadir_angle < NADIR_TOLERANCE_DEG:
                    nadir_locked = True
            else:
                if nadir_angle > NADIR_EXIT_DEG:
                    nadir_locked = False
            
            stable = imu.is_stable()
            
            if not stable:
                log(f"  Skip: not stable (rate={imu.get_angular_rate():.2f})")
                time.sleep(interval); continue
            if not nadir_locked:
                log(f"  Skip: not nadir (angle={nadir_angle:.1f}°)")
                time.sleep(interval); continue
            
            # CAPTURE
            seq = len(images_this_pass)
            fname = f"pass{pass_number}_img{seq:02d}_{timestamp()}.jpg"
            fpath = os.path.join(IMAGE_DIR, fname)
            camera.capture_with_recompress(fpath, JPEG_MAX_SIZE_BYTES)
            
            ang_rate = imu.get_angular_rate()
            cam_meta = camera.get_metadata()
            orientation = imu.get_orientation()
            novelty = coverage.get_novelty(current_grid_cell)
            
            score, status, details = quality_gate.score_image(fpath, ang_rate, novelty)
            
            meta = build_metadata(fname, pass_number, seq, current_grid_cell,
                                  orientation, ang_rate, cam_meta, details, score)
            save_metadata_sidecar(meta, fpath.replace(".jpg", "_meta.json"))
            
            if score > 0:
                tier = "P1" if novelty >= 0.8 else ("P2" if novelty >= 0.3 else "P3")
                meta["priority_tier"] = tier
                meta["downlink_status"] = "pending"
                storage.update_image_index(fname, meta)
                coverage.update(current_grid_cell, score, fname)
                images_this_pass.append(meta)
                log(f"  PASS {fname} score={score:.2f} tier={tier} cell={current_grid_cell}")
            else:
                meta["downlink_status"] = "rejected"
                storage.update_image_index(fname, meta)
                reason = details.get("rejection_reason", "unknown")
                if "blur" in reason: rejections["blur"] += 1
                elif "under" in reason: rejections["underexposed"] += 1
                elif "over" in reason: rejections["overexposed"] += 1
                elif "motion" in reason: rejections["motion_blur"] += 1
                log(f"  REJECT {fname} — {status}")
            
            time.sleep(interval)
        
        log(f"Imaging: {len(images_this_pass)} passed, {sum(rejections.values())} rejected {rejections}")
        
        # === IDLE ===
        log(f"=== PASS {pass_number} — IDLE ===")
        queue = storage.build_priority_queue(pass_number)
        storage.age_images(pass_number)    # [FIX #5] P2 from >2 passes ago → P3
        if storage.is_warning():
            storage.cleanup_p3()
        for cmd in command_listener.get_pending():
            handle_command(cmd, ...)
        storage.save_queue(queue)
        storage.save_image_index()
        coverage.save()
        
        data_budget = THROTTLE_BYTES_PER_SEC * DOWNLINK_WINDOW_SEC  # [FIX #4] = 72000
        log(f"Queue: {len(queue)} images, budget: {data_budget} bytes")
        time.sleep(IDLE_DURATION_SEC)
        
        # === DOWNLINK ===
        log(f"=== PASS {pass_number} — DOWNLINK ===")
        bytes_sent = 0
        images_sent = 0
        
        if gcs_suspended:    # [FIX #11]
            log("GCS suspended — skipping downlink. Images queued locally.")
        else:
            try:
                transfer.connect()
                transfer.send_telemetry(build_telemetry(...))
                
                window_start = time.time()
                while queue and (time.time() - window_start) < DOWNLINK_WINDOW_SEC:
                    watchdog.pet()
                    item = queue[0]
                    
                    if bytes_sent + item["file_size"] > data_budget:
                        log(f"Budget exhausted: {bytes_sent}/{data_budget}, {len(queue)} remain")
                        break
                    
                    ok = transfer.send_file(os.path.join(IMAGE_DIR, item["filename"]), item)
                    if ok:
                        queue.pop(0)
                        storage.mark_sent(item["filename"])
                        bytes_sent += item["file_size"]
                        images_sent += 1
                        log(f"  SENT {item['filename']} ({item['file_size']} B)")
                    else:
                        if transfer.last_error == "NACK":
                            item["retry_count"] = item.get("retry_count", 0) + 1
                            if item["retry_count"] >= MAX_RETRIES_PER_IMAGE:
                                queue.pop(0)
                                storage.mark_failed(item["filename"])
                                log(f"  CORRUPT {item['filename']} — skipping")
                            else:
                                log(f"  NACK — retry {item['retry_count']}")
                        else:
                            log(f"  LINK FAIL — stopping downlink")
                            break
                
                transfer.disconnect()
            
            except GCSUnreachableError:    # [FIX #11]
                gcs_suspended = True
                log("GCS unreachable after 3 attempts — suspending downlink")
        
        # Check if GCS sent a command (means it's back online)
        for cmd in command_listener.get_pending():
            if cmd["cmd"] == "retry_downlink":
                gcs_suspended = False
                log("GCS suspension lifted by ground command")
            handle_command(cmd, ...)
        
        storage.save_queue(queue)
        storage.save_recovery_state({"pass_number": pass_number, ...})
        
        log(f"Downlink: {images_sent} sent, {bytes_sent} bytes, {len(queue)} queued")
        time.sleep(PASS_CYCLE_DELAY_SEC)
```

---

## 9. Design Decision Evidence (Log Entries)

| Log | Proves |
|-----|--------|
| `"Captured 8, rejected 2 (1 blur, 1 motion_blur)"` | On-board quality gating saves bandwidth |
| `"Skip: not stable (rate=1.42)"` | ADCS gates imaging |
| `"Budget exhausted: 56000/72000, 3 remain"` | Data budget is real constraint |
| `"SENT pass3_img07 (28400 bytes)"` + 23-sec transfer | Link budget constrains throughput |
| `"Pass 1 P2 images aged to P3"` | Storage management |
| `"Grid cell → (2, 3)"` | Operator-in-the-loop targeting |
| `"GCS unreachable — suspending downlink"` | Autonomous fault handling |
| `"Recompressed pass3_img05 Q70→Q60 (42KB→29KB)"` | Adaptive compression |
| `"THERMAL WARNING: 72°C — reducing capture rate"` | Real thermal management |
| `"Ground CMD: retransmit pass2_img03"` | Bidirectional comms |

---

## 10. Pre-Demo Calibration Checklist

**[FIX #3] These must be done on the real surface before demo day:**

1. Take 10 sharp photos of the real surface (hold CubeSat still, good focus)
2. Take 10 intentionally blurry photos (shake CubeSat during capture)
3. Run `cv2.Laplacian(gray, cv2.CV_64F).var()` on all 20
4. Record: lowest sharp variance = ___, highest blurry variance = ___
5. Set `BLUR_THRESHOLD` = midpoint between those two values
6. Measure camera-to-surface distance with ruler → `CAMERA_HEIGHT_CM`
7. Measure flashlight angle with protractor → goes in GCS config
8. Measure flashlight direction → goes in GCS config
9. Take USB power meter readings in each mode → fill in `MEASURED_*` values
10. Test solar panel outside in sunlight → record V and A

---

## 11. Testing

| Test | Expected |
|------|----------|
| IMU self-test at rest | Z ≈ -9.8 ± 1.0 |
| Stability: still vs shake | Still → True, Shake → False |
| Nadir: down vs sideways | Down → locked True, Sideways → locked False |
| Nadir hysteresis | Hovering at 44° → stays locked. Cross 55° → unlocks. |
| Blur rejection on real surface | Blurry → rejected. Sharp → passed. Threshold calibrated. |
| Dark rejection | Lens covered → rejected "underexposed" |
| Motion blur | Wave during capture → rejected if angular_rate > 1.0 |
| Adaptive compression | Bright scene → >40KB at Q70 → recompressed to Q60 |
| Throttled transfer 28KB | Takes ~23 real seconds |
| ACK → mark sent | Image removed from queue |
| NACK → retry | Re-sent up to 3 times |
| Link kill mid-transfer | Socket error, image re-queued |
| GCS offline 3 times | Downlink suspended, log says autonomous mode |
| GCS comes back | `retry_downlink` lifts suspension |
| Data budget exhaustion | At ~72 KB, stops sending, logs remainder |
| P2 aging after 2 passes | Pass 1 P2 images → P3 during pass 3 IDLE |
| Operator cell input | `cell 2 3` → captures tagged (2,3) |
| Full 3-pass cycle | All states, queue persists, coverage grows |

---

## 12. Dependencies

```bash
sudo apt install -y python3-picamera2 python3-opencv
sudo pip3 install adafruit-circuitpython-lsm6ds --break-system-packages
sudo pip3 install numpy --break-system-packages
```
