# LumiGraph

**5-bar linkage LED light-drawing machine** — DigiKey Make ONE 2026 entry

A two-device system that traces glyph strokes using a WS2812B LED mounted at the coupler point of a 5-bar linkage mechanism, captured in long-exposure photography.

- **Source code (GitHub)**: https://github.com/sin1n24/LumiGlyphProtoPedia
- **Project page (ProtoPedia)**: https://protopedia.net/prototype/8594

---

## System Overview

| Role | Device | Function |
|------|--------|----------|
| **Drawing** | M5Stack AtomS3R-CAM | Runs inverse kinematics, drives servos, controls LED |
| **Display** | M5Stack AtomS3R | Receives camera feed via ESP-NOW, composites long-exposure image |

The two devices communicate wirelessly over ESP-NOW. The Drawing device streams camera frames to the Display device, which accumulates a per-pixel maximum-brightness composite for light-trail photography.

---

## Hardware

### Drawing device (AtomS3R-CAM)
- **Board**: M5Stack AtomS3R-CAM (ESP32-S3)
- **Servos**: 2× micro servo — G25 (servo 1), G26 (servo 2)
- **LED**: WS2812B × 1 — G0
- **Linkage**: 5-bar planar linkage, all links L = 60 mm, pivot spacing D = 36 mm

### Display device (AtomS3R)
- **Board**: M5Stack AtomS3R (ESP32-S3)
- **Joystick**: analog X → G8, Y → G7
- **Buttons**: Red (menu) → G38, Black (draw) → G39, Trigger (torch) → G5

---

## Build & Flash

Requires [PlatformIO](https://platformio.org/).

```bash
# Flash Drawing device (AtomS3R-CAM)
pio run -e Drawing --target upload

# Flash Display device (AtomS3R)
pio run -e Display --target upload
```

Upload speed is auto-configured per environment in `platformio.ini`.

### First-time pairing
Power on both devices. On the Display, hold BtnA until `PAIRED` appears in the status bar.

---

## Operation

### Display controls

| Input | Action |
|-------|--------|
| **Red button** | Open / close menu |
| **Black button** | Draw selected glyph (in menu: execute) |
| **Trigger** short press | Toggle LED torch on/off |
| **Trigger** long press (≥1 s) | Momentary torch while held |
| **BtnA** double-tap | Set current LED position as home |
| **Joystick** | Manual drawing control / menu navigation |

### Menu items

| Item | Description |
|------|-------------|
| I / Ro / Ha | Single katakana strokes |
| IRoHa | Sequential イ→ロ→ハ, one exposure each |
| IChi | Sequential イ→チ, one exposure each |
| A / Heart / Star / Circle / Tri / Rect | Shape glyphs |
| d / i / g / k / e / y | Lowercase Latin letters |
| **DigiKey** | Draws `D-i-g-i-K-e-y` in sequence |
| **REC** | Record joystick + torch for up to 5 s |
| **PLAY** | Replay last recording |

---

## Glyph System

Glyphs are defined in `src/glyphs.h` as `int8_t` streams:

```
GLYPH_PENUP (-128)  x  y   → lift pen, move to absolute (x, y)
x  y                        → draw to absolute (x, y)
```

Workspace coordinates: `physical_mm = center + value × 0.5`  
Default center: (18, 75) mm — runtime-adjustable via double-tap home.

To generate glyphs from KanjiVG SVG data:

```bash
python tools/convert_kanjivg.py kanjivg/kanji "あいうえお" src/glyphs_data.h
```

`src/glyphs_data.h` (auto-generated) takes priority over `src/glyphs.h` when present.

---

## Servo Calibration

Edit the trim constants at the top of `src/main.cpp`:

```cpp
static int SERVO1_TRIM = 0;   // degrees offset for servo 1
static int SERVO2_TRIM = 0;   // degrees offset for servo 2
static int SERVO1_DIR  = 1;   // 1 or -1
static int SERVO2_DIR  = 1;
```

These values are persisted to SPIFFS and adjustable at runtime via the calibration mode.

---

## Project Structure

```
src/
  main.cpp        # ROLE_DRAWING + ROLE_DISPLAY (selected by build flag)
  glyphs.h        # Hand-authored glyph stroke data
  ik.h            # 5-bar linkage inverse kinematics
tools/
  convert_kanjivg.py   # KanjiVG → glyphs_data.h converter
platformio.ini    # Build environments: Drawing, Display, m5stick-c
```

---

## License

MIT
