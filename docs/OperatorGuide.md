# Operator Guide

## Controller Bindings & Modes

### Driver Controller
- Left Stick: Field-relative drive (translation)
- Right Stick: Field-relative drive (rotation)
- Right Trigger (hold): Hub shot — turret + hood + flywheel lock onto hub, shift-gated feed
- Left Trigger (hold): Pass shot — turret + hood + flywheel track nearest pass target, ungated feed
- Start: Reset gyro heading to 0°

### Operator Controller
- Right Bumper: Toggle intake on/off
- Left Bumper: Stop intake rollers immediately
- A (hold): Agitate balls in intake
- B (hold): Clear shooter system (unjam)
- X (hold): Eject balls from intake rollers
- Y (hold): Run intake rollers inward

## Shooting Modes & Fallbacks

- Left Stick Press: Static Distance (SOTF disabled, turret tracks azimuth)
- Right Stick Press: Full Static (SOTF disabled, turret locked at 0°)
- Default: Full SOTF (lead-compensated, turret tracks azimuth)
- Press again to return to Full SOTF mode.
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
TURRET     →  auto-tracks hub (shift active) or pass target (shift inactive)
FLYWHEEL   →  auto spins 5s before each window
RIGHT TRIGGER (hold)  →  hub shot, shift-gated feed, auto-stops at boundary
LEFT TRIGGER  (hold)  →  pass shot, free feed, tracks nearest pass target
LEDs       →  green wave = go, orange strobe = hurry, dim = wait
```