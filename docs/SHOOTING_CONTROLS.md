# Shooting Controls — Driver & Operator Reference

## Normal Match Flow

### Turret (automatic — no button needed)
The turret **tracks autonomously at all times**:
- When a hub shift is **active** → turret points at the alliance hub center.
- When the shift is **inactive** → turret points at the nearest trench opening.

No driver input is required. The turret will be pre-aimed before you press a trigger.

### Hood (automatic during shots)
The hood returns to **0°** when idle (safe position, avoids the trench bar).
It only raises to the correct shot angle while a trigger is held.

### Flywheel (automatic — no button needed)
The flywheel **pre-spins automatically** when a scoring window is active or within
5 seconds of opening. No driver action required. You will hear/feel it spin up
before the shift starts.

---

## Shot Controls

### Hub Shot — Driver Right Trigger (hold)
- **Hold** → turret and hood lock onto the hub, flywheel tracks exact speed, and
  the robot begins firing balls.
- **Release** → all shooting stops. Turret resumes autonomous tracking.
- Feeding is **shift-gated**: stops automatically when the scoring window closes.
  Hold the trigger through a boundary — feeding resumes when the next window opens
  as long as the trigger is still held.

### Trench / Pass Shot — Driver Left Trigger (hold)
- **Hold** → turret and hood track the nearest trench opening (PASS table), flywheel
  spins to the correct pass speed, and the robot fires.
- **Release** → all shooting stops. Turret resumes autonomous tracking.
- Feeding is **not shift-gated** — fires freely regardless of hub shift state.
- Use this to score into the trench or pass to an alliance partner during or outside
  your shift.

---

## Warning Signals

| Signal | Meaning | Action |
|--------|---------|--------|
| **Right rumble pulse** (1–5 quick pulses) | Scoring window ending in 1–5 seconds | Finish your shots, prepare to release trigger |
| **Both controllers rumble continuously** at teleop start | FMS did not send game data (hub winner unknown) | Alert drive coach — shifts may be wrong |

---

## Operator Controls

| Button | Action |
|--------|--------|
| Right Bumper | Toggle intake on/off |
| Left Bumper | Stop intake rollers immediately |
| A (hold) | Agitate balls in intake |
| B (hold) | Clear shooter system (unjam) |
| X (hold) | Eject balls from intake rollers |
| Y (hold) | Run intake rollers inward |

---

## LED Patterns (Teleop)

The LED strip provides at-a-glance feedback during a match. No driver input is
needed — patterns run automatically based on robot state. See the
[Controller Cheat Sheet](guides/controller-cheat-sheet.md) for the full pattern
list including disabled and autonomous modes.

| Priority | Condition | Pattern |
|----------|-----------|---------|
| 1 | Intaking | Purple strobe (fast) |
| 2 | Aim lock (shooting) | Solid green |
| 3 | Shift ending (≤ 5 s left) | Orange strobe (fast warning) |
| 4 | Shift active | Green / black wave ("go score!") |
| 5 | Shift inactive | Dim alliance color (25%) |

**Read the LEDs like a traffic light:**
- 🟢 **Green wave** = your shift is live, go score
- 🟠 **Orange strobe** = shift is about to change, wrap up
- 🔵 **Dim cyan/red** = not your shift, hold or pass
- 🟣 **Purple strobe** = intake is running
- 🟢 **Solid green** = aim locked, trigger held

---

## Quick Summary

```
TURRET     →  auto-tracks hub (shift active) or pass target (shift inactive)
FLYWHEEL   →  auto spins 5s before each window
RIGHT TRIGGER (hold)  →  hub shot, shift-gated feed, auto-stops at boundary
LEFT TRIGGER  (hold)  →  pass shot, free feed, tracks nearest pass target
LEDs       →  green wave = go, orange strobe = hurry, dim = wait
```

---

## Shooting Modes & Fallbacks

### Shoot Mode Toggles

Operator can toggle between three shooting modes during teleop:

| Stick Press | Mode | Description |
|-------------|------|-------------|
| Left Stick   | Static Distance | SOTF disabled: uses raw hub distance for RPM/hood, turret tracks azimuth |
| Right Stick  | Full Static     | SOTF disabled: raw hub distance AND turret locked at 0° |
| (Default)    | Full SOTF       | Lead-compensated distance for RPM/hood, turret tracks azimuth |

Pressing either stick toggles the mode; press again to return to Full SOTF. Mode is logged to NetworkTables as `ShooterTelemetry/shootMode`.

### RPM Fudge Factor

Operator can adjust flywheel RPM mid-match using dashboard key `Shooter/RPMFudgePercent`.
This applies a percentage offset to the computed RPM:
- Positive = more RPM (shots short)
- Negative = less RPM (shots long)
- Default is 0%. Changes take effect immediately.

---

## Fallback Logic

If SOTF is unreliable or disabled, fallback modes ensure robust shooting:
- Static Distance: Uses raw hub distance for interpolation, turret still tracks.
- Full Static: Uses raw hub distance, turret locked at 0° (comp-proven fallback).
All modes are accessible via operator stick toggles for rapid mid-match adjustment.
