# Controller Guide

## Competition Mode (default)

Two controllers — **Driver** (port 0) owns driving and shooting, **Operator** (port 1)
owns intake, shoot-mode selection, and secondary overrides.

### Driver Controller (Port 0)

| Button | Action |
|--------|--------|
| Left Stick | Field-relative drive (translation) |
| Right Stick | Field-relative drive (rotation) — auto-rotates toward hub when RT > 50 % |
| Right Trigger (hold) | **Hub shot** — hybrid aim + shoot (turret clamped −90 ° / +130 °, drivetrain heading assist, shift-gated feed) |
| Left Trigger (hold) | **Pass shot** — hybrid aim + shoot at nearest pass target (turret clamped, ungated feed) |
| Start | Reset gyro heading to 0° |

### Operator Controller (Port 1)

| Button | Action |
|--------|--------|
| Right Trigger (hold) | Intake (deploy arm + run rollers) |
| Left Trigger (hold) | Eject (reverse intake rollers) |
| A (hold) | Agitate (re-deploy arm + jog rollers to dislodge stuck balls) |
| B (hold) | Clear / unjam shooter system |
| X (press & hold) | Defence mode LEDs (red/blue strobe while held) |
| Y (press) | Toggle shoot mode: FULL ↔ FULL_STATIC |
| Right Bumper (press) | +50 RPM fudge (clamped to +200) |
| Left Bumper (press) | −50 RPM fudge (clamped to −200) |

---

## Tuning Mode (`Constants.tuningMode = true`)

**Single driver controller only** — designed for one person testing in the pit.
No operator controller is required.

### Driver Controller (Port 0)

| Button | Action |
|--------|--------|
| Left Stick | Field-relative drive (translation) |
| Right Stick | Field-relative drive (rotation) — auto-rotates toward hub when RT > 50 % |
| Right Trigger (hold) | **Aim only** — hybrid aim at hub (turret + hood, no flywheel or feed) |
| Left Trigger (hold) | Intake (deploy arm + run rollers) |
| D-pad Down (hold) | Eject (reverse intake rollers) |
| D-pad Up (hold) | Turret snap → 0° (home) |
| D-pad Left (hold) | Turret snap → 90° |
| D-pad Right (hold) | Turret snap → −90° |
| D-pad Up-Left (hold) | Turret snap → 45° |
| D-pad Up-Right (hold) | Turret snap → −45° |
| A (hold) | Agitate |
| B (hold) | Clear / unjam shooter system |
| X (hold) | **Test shoot** — turret at 0 °, spin flywheel + feed (fires without vision) |
| Y (press) | Toggle shoot mode: FULL ↔ FULL_STATIC |
| Right Bumper (press) | +50 RPM fudge |
| Left Bumper (press) | −50 RPM fudge |
| Start | Reset gyro heading to 0° |

---

## Shooting Modes

| Mode | Behaviour |
|------|-----------|
| **FULL** (default) | Full SOTF — turret auto-tracks hub azimuth, lead-compensated moving shot solver, hood + flywheel from lookup table. |
| **FULL_STATIC** | Turret locked at 0° (facing backwards), no lead compensation. Manual fallback for when vision is unreliable. |

Toggle between modes with **Y** (operator in competition, driver in tuning).
Mode is logged to `ShooterTelemetry/shootMode`.

---

## RPM Fudge Factor

- Dashboard key: `Tuning/Shooter/RPMFudge` (absolute RPM offset, **not** a percentage).
- Adjusted via **RB / LB** in ±50 RPM increments, clamped to ±200 RPM.
- Button presses write through to the dashboard slider in real time.
- Positive = more RPM (compensates for short shots), negative = less RPM.
- Default is 0. Changes take effect immediately.

---

## Default Behaviours (when no command is active)

| Subsystem | Default |
|-----------|---------|
| Turret | Stow at 0° (`Turret_DefaultStow`) |
| Hood | Stow at 0° (`Hood_DefaultStow`) |
| Flywheel | Stopped (no pre-spin) |
| Kicker | Stopped |
| Spindexer | Stopped (coast) |
| Intake Rollers | Stopped |
| Intake Pivot | Hold at 0 |
| Drive | Manual sticks; heading assist toward hub when RT > 50 % |

---

## LED Patterns

| Priority | Condition | Pattern |
|----------|-----------|---------|
| 1 (highest) | Defence mode (operator X held) | Red / blue strobe |
| 2 | Intaking | Purple strobe (fast) |
| 3 | Aim lock (RT or LT held) | Solid green |
| 4 | Shift ending (≤ 5 s remaining) | Orange strobe (fast warning) |
| 5 | Shift active | Green / black wave ("go score!") |
| 6 (lowest) | Shift inactive | Dim alliance colour (25 %) |

**Read the LEDs like a traffic light:**
- �🔵 Red/blue strobe = defence mode active
- 🟣 Purple strobe = intake is running
- 🟢 Solid green = aim locked, trigger held
- 🟠 Orange strobe = shift is about to change, wrap up
- � Green wave = your shift is live, go score
- �🔵 Dim cyan/red = not your shift, hold or pass

---

## Rumble Alerts

| Trigger | Controller | Pattern |
|---------|-----------|---------|
| Shift ending (≤ 5 s) | Operator right rumble | 250 ms pulse each second |
| No hub-winner data from FMS after 1 s of teleop | Both controllers | Continuous rumble until data arrives |

---

## Quick Summary

```
COMPETITION — DRIVER (port 0):
  Left / Right Stick    →  field-relative drive (heading assist when RT held)
  Right Trigger (hold)  →  hub shot (hybrid aim + shoot, shift-gated feed)
  Left Trigger  (hold)  →  pass shot (hybrid aim + shoot, ungated feed)
  Start                 →  reset gyro to 0°

COMPETITION — OPERATOR (port 1):
  Right Trigger (hold)  →  intake
  Left Trigger  (hold)  →  eject
  A (hold)              →  agitate
  B (hold)              →  clear / unjam shooter
  X (hold)              →  defence mode LEDs
  Y (press)             →  toggle FULL ↔ FULL_STATIC
  RB / LB (press)       →  ±50 RPM fudge

TUNING — DRIVER ONLY (port 0):
  Left / Right Stick    →  field-relative drive (heading assist when RT held)
  Right Trigger (hold)  →  aim only (turret + hood, no feed / flywheel)
  Left Trigger  (hold)  →  intake
  D-pad Down (hold)     →  eject
  D-pad Up (hold)       →  turret snap 0°
  D-pad Left (hold)     →  turret snap 90°
  D-pad Right (hold)    →  turret snap −90°
  D-pad Up-Left (hold)  →  turret snap 45°
  D-pad Up-Right (hold) →  turret snap −45°
  A (hold)              →  agitate
  B (hold)              →  clear / unjam shooter
  X (hold)              →  test shoot (turret 0°, flywheel + feed, no vision)
  Y (press)             →  toggle FULL ↔ FULL_STATIC
  RB / LB (press)       →  ±50 RPM fudge
  Start                 →  reset gyro to 0°

DEFAULTS (always active):
  Turret    →  stow at 0°
  Hood      →  stow at 0°
  Flywheel  →  stopped (no pre-spin)
  LEDs      →  defence > intaking > aim lock > shift patterns
  Rumble    →  operator pulses in last 5 s of shift
```
