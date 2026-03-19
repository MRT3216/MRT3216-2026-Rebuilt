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

## Quick Summary

```
TURRET     →  auto-tracks hub (shift active) or trench (shift inactive)
FLYWHEEL   →  auto spins 5s before each window
RIGHT TRIGGER (hold)  →  hub shot, shift-gated feed, auto-stops at boundary
LEFT TRIGGER  (hold)  →  trench/pass shot, free feed, tracks trench opening
```
