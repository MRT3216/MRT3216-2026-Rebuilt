# Controller Guide

## Controller Bindings & Modes

### Driver Controller (Port 0) — Driving & Intake

| Button | Action |
|--------|--------|
| Left Stick | Field-relative drive (translation) |
| Right Stick | Field-relative drive (rotation) |
| Right Trigger | Toggle intake on/off (press once to start, press again to cancel) |
| Left Trigger | Stop intake rollers immediately |
| Right Bumper (hold) | Agitate balls in intake; deploys intake on release |
| Left Bumper (hold) | Eject balls from intake rollers |
| Start | Reset gyro heading to 0° |

### Operator Controller (Port 1) — Shooting & Overrides

| Button | Action |
|--------|--------|
| Right Trigger (hold) | Hub shot — turret + hood + flywheel lock onto hub, shift-gated feed |
| Left Trigger (hold) | Pass shot — turret + hood + flywheel track nearest pass target, ungated feed |
| Right Bumper (hold) | Run intake rollers inward (manual override) |
| Left Bumper (hold) | Clear / unjam shooter system |
| Left Stick Press | Toggle Static Distance shoot mode |
| Right Stick Press | Toggle Full Static shoot mode |

### Shooting Modes & Fallbacks (Operator)

- **Full SOTF** (default): Lead-compensated, turret tracks azimuth + moving shot solver.
- **Static Distance**: SOTF disabled, turret still tracks azimuth but uses a fixed distance model. Toggle with left stick press.
- **Full Static**: SOTF disabled, turret locked at 0° (backwards). Battle-tested comp fallback. Toggle with right stick press.
- Press the same stick again to return to Full SOTF mode.
- Mode is logged to NetworkTables as `ShooterTelemetry/shootMode`.

## RPM Fudge Factor

- Dashboard key `Shooter/RPMFudgePercent` applies a percentage offset to computed RPM.
- Positive = more RPM (shots short), negative = less RPM (shots long).
- Default is 0%. Changes take effect immediately.

## LED Patterns

| Priority | Condition | Pattern |
|----------|-----------|---------|
| 1 | Intaking | Purple strobe (fast) |
| 2 | Aim lock (shooting) | Solid green |
| 3 | Shift ending (≤ 5 s left) | Orange strobe (fast warning) |
| 4 | Shift active | Green / black wave ("go score!") |
| 5 | Shift inactive | Dim alliance color (25%) |

**Read the LEDs like a traffic light:**
- 🟢 Green wave = your shift is live, go score
- 🟠 Orange strobe = shift is about to change, wrap up
- 🔵 Dim cyan/red = not your shift, hold or pass
- 🟣 Purple strobe = intake is running
- 🟢 Solid green = aim locked, trigger held

## Quick Summary

```
DRIVER:
  Right Trigger       →  toggle intake on/off
  Left Trigger        →  stop intake rollers
  Right Bumper (hold) →  agitate balls, deploy on release
  Left Bumper (hold)  →  eject balls

OPERATOR:
  Right Trigger (hold) →  hub shot, shift-gated feed, auto-stops at boundary
  Left Trigger  (hold) →  pass shot, free feed, tracks nearest pass target
  Right Bumper (hold)  →  manual intake rollers inward
  Left Bumper (hold)   →  clear / unjam shooter

AUTOMATIC:
  TURRET    →  auto-tracks hub (shift active) or pass target (shift inactive)
  FLYWHEEL  →  auto spins 5s before each shift window
  LEDs      →  green wave = go, orange strobe = hurry, dim = wait
  RUMBLE    →  both controllers pulse in last 5s of shift
```
