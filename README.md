# MuraltZ CubeSat — Flight Software

Raspberry Pi flight software for a CubeSat prototype. It gates camera captures on live IMU stability and nadir pointing, scores every image on-board for quality and novelty, and downlinks the highest-priority images over a deliberately throttled link. Built at the MIT Beaver Works Summer Institute.

The companion ground station lives in [MIT-Cubesat-Ground-Control-Station](https://github.com/ayushg8/MIT-Cubesat-Ground-Control-Station).

## Overview

A real satellite cannot photograph everything and cannot downlink everything. Power, pointing, and link budget force it to choose. This flight software makes those choices on-board:

- It only takes a picture when the spacecraft is actually pointed at the ground and holding still.
- It rejects blurred, over/under-exposed, and redundant frames before they take up storage or link time.
- It ranks what survives and sends the most valuable images first over a link slow enough that selection is mandatory, not optional.

The link is throttled with a real `time.sleep` per chunk, so the constraint the spacecraft reasons about is genuinely present at runtime rather than assumed away.

## How it works

The flight loop is an operator-stepped state machine (`cubesat_flight/main.py`). After boot it waits for a pass command, then cycles **IMAGING → IDLE → DOWNLINK** and returns to waiting.

```
BOOT ──▶ WAITING ──▶ IMAGING ──▶ IDLE ──▶ DOWNLINK ──▶ WAITING
            ▲                                              │
            └──────────────────────────────────────────────┘
```

**BOOT** — IMU sanity check, a test capture that must decode as a JPEG, a disk-capacity check, and a ground-station connection attempt. A missing ground station is non-fatal: the spacecraft drops into autonomous mode and keeps imaging.

**IMAGING** (`states/imaging.py`) — every few seconds it reads the IMU and captures only when **both** conditions hold:
- **Nadir lock** — pointing angle within 45° of nadir, with hysteresis (the lock holds until the angle exceeds 55°) to stop chattering at the boundary.
- **Stability** — gyro magnitude below 1.0 rad/s.

Each captured frame runs a quality gate (`processing/quality.py`) on four axes — blur (Laplacian variance), exposure (mean-pixel bounds), motion (gyro rate at capture time), and novelty — and is either accepted with a priority tier (P1/P2/P3, assigned by novelty against an 8×8 coverage grid) or rejected with a logged reason.

**IDLE** (`states/idle.py`) — builds the downlink priority queue, ages stale images down a tier, deletes the lowest-priority images when storage runs high, and applies any queued ground commands.

**DOWNLINK** (`states/downlink.py`, `comms/transfer.py`) — sends telemetry, then images highest-priority first in 1200-byte chunks, each followed by a real one-second sleep, until the per-pass byte budget or time window is spent. Every image carries an MD5 header; the receiver ACK/NACKs each one, with up to three retries before an image is marked corrupt.

Supporting subsystems: a watchdog thread that saves recovery state and restarts the process if the main loop stalls, thermal monitoring that throttles imaging at 70 °C and drops to safe mode at 80 °C, and a command listener that accepts validated JSON commands from the ground station over TCP.

### Offline analysis layer

The repo also contains a hardware-independent image-analysis library — pixel-level terrain segmentation, an A\* route planner over a cost grid, and a Flask inspection dashboard (`processing/pipeline.py`, `processing/route_planner.py`, `processing/dashboard/`). This runs offline and in the test suite; it is **not** wired into the in-flight loop. It is exercised by `test_processing.py` in CI.

## Tech

- **Python 3** on **Raspberry Pi 4**
- **LSM6DSO32 IMU** over I²C (Adafruit CircuitPython) — accelerometer + gyro; no magnetometer, so yaw is not estimated
- **Pi Camera Module 3** via `picamera2`
- **OpenCV** / **NumPy** for the quality gate and offline analysis
- **Flask** for the offline inspection dashboard
- On-board Wi-Fi stands in for a UHF radio; CPU temperature read from sysfs

## Running it

On the Pi (hardware):

```bash
sudo apt install -y python3-picamera2 python3-opencv
pip install -r requirements-hardware.txt
cd cubesat_flight
python test_hardware.py     # IMU / camera / thermal / storage / link self-check
python main.py              # then type: start_pass, cell R C, end_pass, status, shutdown
```

On a laptop (no hardware — runs the offline analysis tests):

```bash
pip install -r requirements.txt
cd cubesat_flight
export CUBESAT_DATA_ROOT="$(mktemp -d)"
python -m unittest -v test_processing.py
```

`GROUND_STATION_IP` and the data root are environment-overridable (see `.env.example`).

## Design constraints and honest limits

This is an educational prototype, and the code is explicit about what is and isn't calibrated:

- The blur threshold is a placeholder that must be calibrated against the real imaging surface before a demo; it is flagged as such in `config.py`.
- There is no magnetometer, so the spacecraft tracks pitch and roll but not yaw.
- Wi-Fi substitutes for a flight radio; the 1200 byte/s throttle reproduces a 9600-baud UHF link.
- The values below are real, designed behaviors in the code — not measured flight results.

| Behavior | Value (default) |
| --- | --- |
| Stability gate (gyro) | < 1.0 rad/s |
| Nadir lock / release | 45° / 55° (hysteresis) |
| Blur gate (Laplacian variance) | < 20.0 (uncalibrated) |
| Exposure gate (mean pixel) | 15–240 |
| Link throttle | 1200 bytes/s (real per-chunk sleep) |
| Per-pass budget | 72,000 bytes / 60 s window |
| Thermal throttle / safe mode | 70 °C / 80 °C |
| Watchdog restart | main-loop stall > 30 s |

## TODO — visual assets to add

The repo has no usable hardware or demo imagery yet. Add and reference here:

- `docs/img/hardware.jpg` — photo of the assembled Pi + camera + IMU prototype.
- `docs/img/imaging-demo.gif` — a short clip showing frames accepted/rejected as the rig moves (motion blur → reject, held still on target → accept).
- `docs/img/priority-queue.png` — terminal screenshot of a downlink pass sending P1 images first.

(The existing `Images/` folder holds raw, unlabeled test captures and is not linked from this README.)
