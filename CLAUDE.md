# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project overview

Drone-competition rectangle-target tracking system (电赛E题). A K230-CanMV board performs computer vision **and drives the gimbal directly** — no intermediate microcontroller. Both axes use Emm42_V5.0 closed-loop stepper drivers with 42 steppers:

- **yaw (left/right)** — UART3, serial **speed mode**: `|dx|` linearly mapped to RPM.
- **pitch (up/down)** — UART2, serial **speed mode**: identical logic to yaw (`|dy|` → RPM).
  (A position-mode path exists in `EmmStepper` — `move`/`clear_position`/`home` — but is NOT
  wired into the control flow: driver ② failed with it on hardware. Both axes are now the
  same speed-mode logic.)
- **V7 features** — joint rect+ring detection (tracking aims the rectangle center; the
  earlier red-dot/bullseye aim was removed as it conflicted with rect detection — ring
  detection feeds circle mode only), ring-circling gated by GPIO49/Pin3 (powered = circle),
  and boot search (yaw spins single-direction until first acquisition). All params live in
  `k230/config.json` (`[red_target]` / `[search]` / `[circle]` / `[mode_pin]`).
- **Touchscreen center calibration (V7.7)** — on-screen green dot + 6 touch buttons (▲▼◀▶ move
  the aim offset live, ● hides the UI, S saves `center.laser_offset_x/y` back to config.json);
  touch failure auto-disables without affecting tracking. Params in `[center_calib]`.

(The original MaixCam + STM32F407 relay architecture is retired — there is no `stm32/` directory and no `fs_servo.py`; all code is single-file in `k230/main.py`.)

## Architecture

```
K230-CanMV (vision + gimbal control)
    MIPI CSI camera (GC2093)     UART3 → Emm42 driver ① → 42 stepper   [yaw · speed mode]
    LCD ST7701 800×480           UART2 → Emm42 driver ② → 42 stepper   [pitch · speed mode]
    Optional laser GPIO          GPIO49 (Pin3) mode jumper: 3.3V = circle mode, GND = track-only
```

Visual→actuator interface: `gimbal_send(dx, dy, dist, status)` inside `k230/main.py` (section 四).

Camera capture: onboard GC2093 sits on **CSI2** — `Sensor(id=CSI_ID, …)` reads `camera.csi_id: 2`
from config. Probe resolution **1280×960 (4:3) must match the 640×480 output** — a 1280×720
(16:9) probe stretched the image horizontally (V7.7b). Do NOT drop the id — missing id once
caused a view-offset (画面偏左) regression.

## Repository structure

```
复刻/
├── k230/
│   ├── main.py          # single-file runtime: config load → EmmStepper driver (speed+position)
│   │                    #   → actuator init + gimbal_send → CV/YOLO detection → main loops
│   └── config.json      # the sole tuning surface ([motor] = yaw, [pitch_motor] = pitch)
├── 接线图.md             # authoritative wiring + driver preset tables
├── skill/               # methodology docs (reference only, not runtime)
└── tools/
    ├── calibrate_laser.py      # laser/optical offset calibration
    ├── test_emm_protocol.py    # host-side frame test (AST-extracts EmmStepper from main.py)
    ├── test_coast_logic.py     # coast state-machine test (AST-extracts coast_step)
    ├── test_circle_logic.py    # V7 pure-function tests (priority/circle/search/pin)
    ├── test_camera_init.py     # capture guard: CSI id + 1280×960 probe (geometry regression)
    └── test_center_calib.py    # touch-calibration: hit test / touch state machine / config save
```

## Critical performance constraints (MicroPython on K230)

MicroPython on K230 has no JIT. These anti-patterns cause 5-10x slowdowns on the hot path:

1. **Class attribute access** (`self.xxx`) — costs 5-10x more than reading a module-level global.
2. **Hot-path `if` branches** — each branch check per frame adds up.
3. **Dict allocation** — triggers GC, causing periodic frame drops.

The codebase uses a **lambda injection pattern** to eliminate hot-path branching. Optional features (Kalman filter, UART) are resolved at import time:

```python
# At module load — one-time decision:
if KALMAN_ENABLED:
    _kf = _Kalman(KALMAN_ALPHA)
    kf_update = _kf.update       # real EMA
    kf_reset  = _kf.reset
else:
    kf_update = lambda mx, my: (float(mx), float(my))  # pass-through
    kf_reset  = lambda: None                            # no-op

# In the hot loop — no branch:
fdx, fdy = kf_update(dx, dy)  # works identically whether Kalman is on or off
```

**Never add an `if FEATURE_ENABLED:` inside the main `while True` loop.**

## Config system

- `k230/config.json` is the **only tuning surface**. Sections: detection, camera, display, center, center_calib, rectangle, yolo, motor (yaw), pitch_motor (pitch), red_target, search, circle, mode_pin, debug.
- The config loader strips keys starting with `_`, so comments can be embedded as `"_说明": "..."`.
- Values are read at import time into module-level globals; restart `main.py` after edits.

## Emm42 protocol essentials (k230/main.py, section 三)

- Frame: `[addr][func][data...][checksum]`; checksum is the fixed value **0x6B**; all
  multi-byte fields are **big-endian** (opposite of Modbus).
- Used commands:
  - enable `F3 AB <en> <sync>`, stop `FE 98 <sync>`
  - speed mode `F6 <dir> <spd×2> <acc> <sync>` (yaw)
  - position mode `FD <dir> <spd×2> <acc> <pulses×4> <abs> <sync>` (reserved — not wired into control flow; abs=01 absolute, negative targets two's-complement)
  - position clear `0A 6D` (defines current physical position as 0° — pitch boot)
  - home `9A <mode> <sync>` (reserved, not wired into the control flow)
  - read bus voltage `24` (online probe)
- 16 microsteps → 3200 pulses/rev (only relevant to the reserved position-mode path).
- Driver-side presets (checksum 0x6B, 115200, UART_FUN, Response=Receive, ID) — see `接线图.md`.

## Testing

- **Host-side (no hardware)**: five stdlib-only scripts, all static-check `k230/main.py`
  (some also read `k230/config.json`):
  - `python tools/test_emm_protocol.py` — frame bytes vs manual examples (12 cases). Always run
    after touching protocol code.
  - `python tools/test_coast_logic.py` — coast (lost-target pre-judgment) state machine (10 cases).
  - `python tools/test_circle_logic.py` — V7 pure functions (29 cases): quad containment, ring
    blob classify, circle geometry, search & pin-debounce machines. Threshold constants are
    extracted from main.py itself — a missing definition is a hard error.
  - `python tools/test_camera_init.py` — capture guard (5 cases): explicit `id=CSI_ID`, 1280×960
    probe (4:3, matches output), `CSI_ID` defined in main.py, `camera.csi_id=2` in config.
  - `python tools/test_center_calib.py` — touch calibration (32 cases): button hit test, touch
    state machine (wake edge / repeat / no-bounce on hide), config text save (exactly 2 lines).
- **Hardware**: `motor.test_mode` / `pitch_motor.test_mode` in config.json perform power-on
  self-checks (online probe + continuous rotation) without running vision. `main.py` also
  self-reports at boot: the `actuator:` line (both axes + online voltage).

## Detection modes

- **CV** (default): Classic OpenCV pipeline — white mask → contours → black-border/white-center
  verification → scoring. ~45fps, no model needed. Only supports white rectangle targets on dark backgrounds.
- **YOLO**: KPU inference via PipeLine/AIBase/Ai2d framework. Requires `model.kmodel` on SD card. ~25fps.
  `input_size` must equal training `imgsz`; `task` (detect/segment) must match the model.

## OpenCV speed optimization techniques used

| Layer | Strategy |
|-------|----------|
| Input | Detection downsampling (320×240, 1/4 pixels), frame skipping (`detect_every`) |
| Preprocess | `cv2.inRange` color mask (binary image directly, faster than grayscale+Canny) |
| Contour | `RETR_EXTERNAL` (outer contours only), loose `approxPolyDP` (epsilon=0.04) |
| Filter | Area + aspect-ratio boundingRect (O(1)), before expensive contour analysis |
| Temporal | Spatial continuity filter (center jump / area ratio), lost-frame hold |
| Verify | ROI-only black-border and white-center checks (not full-image) |
| Runtime | GC every N frames, print every N frames (both configurable) |

## UART pin map (standard edition 40-pin header)

| Axis | UART | TX | RX | Driver |
|---|---|---|---|---|
| yaw | UART3 | GPIO32 (Pin37) | GPIO33 (Pin40) | ① (speed mode) |
| pitch | UART2 | GPIO5 (Pin11) | GPIO6 (Pin13) | ② (speed mode) |

One UART controller can only own one pin set — never configure both axes on the same `port`
(the program prints a WARN at boot if it happens). Full wiring in `接线图.md`.
