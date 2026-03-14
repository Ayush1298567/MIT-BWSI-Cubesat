# MuraltZ CubeSat Flight Software

## What This Is
Flight software for a Raspberry Pi 4 (2GB) CubeSat prototype for the MIT BWSI Build-a-CubeSat Challenge 2025-2026. The mission is Artemis Lunar Navigator — capturing images of a simulated lunar surface and processing them for terrain hazard detection.

## Hardware (CONFIRMED — do not change these)
- **Flight computer:** Raspberry Pi 4 (2GB), hostname `cubesat`, user `cubesat`
- **IMU:** LSM6DSO32 at I2C address 0x6A. Import: `from adafruit_lsm6ds.lsm6dso32 import LSM6DSO32 as LSM6DS`. NOT lsm6ds33, NOT lsm6dsox.
- **Camera:** Pi Camera Module 3 (IMX708). Import: `from picamera2 import Picamera2`. NOT libcamera. CLI: `rpicam-still`.
- **Comms:** Built-in WiFi to ground station laptop
- **No magnetometer in use** (device at 0x1C is unused). Cannot compute yaw — only roll/pitch from accelerometer.

## Architecture Reference
See `docs/ARCHITECTURE.md` for the full software architecture, config parameters, module specs, state machine, protocol contract, and pseudocode.

## Key Rules
1. Everything is REAL. No simulated data, no fake battery levels, no mock sensors. Every value comes from real hardware.
2. Grid cell assignment comes from operator text input (`cell R C`), NOT from elapsed time or trajectory maps.
3. Camera runs in AUTO-EXPOSURE mode by default. No manual exposure calibration.
4. WiFi transfers are REAL but throttled to 1200 bytes/sec (matching UHF flight equivalent). This throttling uses real `time.sleep()` — transfers genuinely take ~23 seconds for a 28 KB image.
5. Data budget per downlink window = `THROTTLE_BYTES_PER_SEC * DOWNLINK_WINDOW_SEC` = 72,000 bytes. NOT a separate 2 MB constant.
6. The state machine waits for operator input (`start_pass`) between passes — it does NOT auto-loop.
7. IMU nadir check uses hysteresis: lock at < 45°, unlock at > 55°. Prevents toggling.
8. Blur threshold default (50) MUST be calibrated against the real surface before demo.

## Dependencies
```bash
sudo apt install -y python3-picamera2 python3-opencv
sudo pip3 install adafruit-circuitpython-lsm6ds --break-system-packages
sudo pip3 install numpy --break-system-packages
```

## File Structure Target
```
cubesat_flight/
├── main.py
├── config.py
├── protocol.py
├── states/
├── sensors/
├── processing/
├── comms/
├── storage/
└── utils/
```
See docs/ARCHITECTURE.md for full tree with all modules.
