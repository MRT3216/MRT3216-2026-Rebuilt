# Shooting Controls — Driver & Operator Reference

## Normal Match Flow

### Flywheel (automatic — no button needed)
The flywheel **pre-spins automatically** when a scoring window is active or within
5 seconds of opening. No driver action required. You will hear/feel it spin up
before the shift starts.

### Starting a Shot — Driver Right Trigger (toggle)
- **Press once** → begins aiming and feeding. The robot locks onto the hub,
  adjusts the hood, and starts firing balls.
- **Press again** → cancels shooting and returns to idle.
- This is a **toggle**, not a hold. You do not need to keep the trigger pulled.

### Automatic Shift Stop
When the scoring window **closes**, the kicker and spindexer stop automatically —
no driver input needed. The flywheel and hood stay active and ready.

> ⚠️ **Important:** Feeding does **not** automatically resume when the next window
> opens. You must press the right trigger again at the start of each new shift.

### Emergency Stop — Driver Left Trigger
- Press once → **immediately cancels** all shooting activity (flywheel, kicker,
  spindexer, hood, turret).
- Use this if the robot gets stuck in a shooting state or you need to stop quickly.
- To resume shooting after an emergency stop, press Right Trigger again.

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

## Warning Signals

| Signal | Meaning | Action |
|--------|---------|--------|
| **Right rumble pulse** (1–5 quick pulses) | Scoring window ending in 1–5 seconds | Finish your shots, prepare to stop |
| **Both controllers rumble continuously** at teleop start | FMS did not send game data (hub winner unknown) | Alert drive coach — shifts may be wrong |

---

## Quick Summary

```
FLYWHEEL  →  auto spins 5s before each window
RIGHT TRIGGER (toggle) →  start shooting
SHIFT ENDS  →  kicker/spindexer stop automatically, flywheel stays on
RIGHT TRIGGER (toggle) →  press again to resume next window
LEFT TRIGGER  →  emergency full stop
```
