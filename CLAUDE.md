# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project overview

Dual-chip rectangle-target tracking system for a drone competition (电赛E题). A K230-CanMV board handles computer vision, sending pixel偏差 over UART to an STM32F407 that drives a 2-axis stepper-motor gimbal via PID closed-loop control.

## Architecture

```
K230-CanMV (visual processing)  ──UART "dx,dy,dist,status\n"──>  STM32F407 (PID gimbal)
    MIPI CSI OV5640 camera                                        UART3 → X-axis stepper
    LCD ST7701 800×480                                            UART6 → Y-axis stepper
    Optional laser GPIO
```

**Key departure from the original MaixCam project:** closed-loop PID tracking replaces open-loop fixed-angle rotation. CV detection with optional YOLO KPU replaces NN-only detection.

## Repository structure

```
复刻/
├── k230/
│   ├── main.py          # K230 runtime — CV + YOLO dual mode, ~690 lines
│   ├── fs_servo.py      # FashionStar bus servo driver (FSUS protocol over UART, servo.enabled)
│   └── config.json      # Runtime config — the sole tuning surface, no code edits needed
├── stm32/
│   ├── README.md        # Build instructions (HAL files to copy, GPIO config)
│   └── User/            # Application code: main.c, pid.c/h, DATOU.c/h, frame.c/h, Key.c/h
├── skill/               # Methodology docs and training guides (reference only, not runtime)
├── tools/
│   └── calibrate_laser.py  # Laser/optical offset calibration
└── README.md
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

**The same pattern is used for `uart_send`.** Never add an `if FEATURE_ENABLED:` inside the main `while True` loop.

## Config system

- `k230/config.json` is the **only tuning surface**. All parameters (11 sections: detection, camera, detect_resolution, center, rectangle, yolo, kalman, tracking, uart, display, debug) are read at import time into module-level globals.
- `skill/config_template.yaml` is a **reference document** with full Chinese annotations — update it whenever config keys change.
- The config loader strips keys starting with `_`, so you can embed comments as `"_说明": "..."`.

## UART protocol (K230 → STM32)

```
Format:  "dx,dy,dist,status\n"

Normal tracking:  "-25,18,150,0\n"     status=0
Aligned:          "-2,1,5,1\n"         status=1
Target lost:      "404,404,0,0\n"      status=404

dist = sqrt(dx² + dy²)   # Euclidean pixel distance to screen center
```

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

## Detection modes

- **CV** (default): Classic OpenCV pipeline — white mask → contours → black-border/white-center verification → scoring. ~45fps, no model needed.
- **YOLO**: KPU inference via PipeLine/AIBase/Ai2d framework. Requires `model.kmodel` on SD card. ~25fps.
- **Hybrid**: CV-first, falls back to history hold on consecutive failures.

## STM32 side

See `stm32/README.md` for build steps. The STM32 uses dual-PID (coarse/fine) with a pixel-deviation threshold switch. Original project files (DATOU, frame, Key) are preserved; only `main.c` was rewritten for PID closed-loop and `pid.c/h` were added.
