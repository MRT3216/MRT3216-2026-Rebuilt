# Controller Cheat Sheet

Quick reference for all Xbox controller bindings and LED patterns.

---

## Competition Mode (Real Bindings)

### Automatic Behavior (no button needed)

| System | Behavior |
|--------|----------|
| **Turret** | Tracks hub center when shift is active; tracks nearest pass target when inactive |
| **Flywheel** | Pre-spins automatically when shift is active or within 5 s of starting |
| **Hood** | Returns to 0° when idle (safe under trench) |
| **Kicker / Spindexer** | Stopped (hold at zero) when idle |
| **Intake rollers** | Stopped when idle |

### Driver Controller

| Input | Action |
|-------|--------|
| **Left Stick** | Field-relative drive (translation) |
| **Right Stick** | Field-relative drive (rotation) |
| **Right Trigger** (hold) | **Hub shot** — turret + hood + flywheel lock onto hub, shift-gated feed |
| **Left Trigger** (hold) | **Pass shot** — turret + hood + flywheel track nearest pass target, ungated feed |
| **Start** | Reset gyro heading to 0° |

### Operator Controller

| Input | Action |
|-------|--------|
| **Right Bumper** | Toggle intake on/off |
| **Left Bumper** | Stop intake rollers immediately |
| **A** (hold) | Agitate balls in intake; deploys on release |
| **B** (hold) | Clear / unjam shooter system |
| **X** (hold) | Eject balls from intake rollers |
| **Y** (hold) | Run intake rollers inward |

### Warning Signals

| Signal | Meaning | Action |
|--------|---------|--------|
| **Right rumble pulse** (1–5 quick pulses) | Scoring window ending in 1–5 seconds | Finish shots, prepare to release trigger |
| **Both controllers rumble continuously** at teleop start | FMS did not send game data (hub winner unknown) | Alert drive coach — shifts may be wrong |

---

## Test / Tuning Mode Bindings

Active when `Constants.tuningMode = true`. See the
[Shooter Calibration Guide](shooter-calibration.md) for the full testShoot workflow.

### Driver Controller

| Input | Action |
|-------|--------|
| **A** (hold) | Aim turret at hub (tracking only, no shoot) |
| **B** (hold) | Run intake rollers at target velocity |
| **X** (hold) | Run spindexer feed |
| **Y** (hold) | Run kicker feed |
| **Left Bumper** (hold) | Deploy intake pivot briefly |
| **Right Bumper** (hold) | Agitate intake |
| **Right Trigger** (hold) | **testShoot** — full calibration routine (see guide) |
| **Right Stick** | Manual turret steering (stick angle → turret angle) |
| **Start** | Reset gyro heading to 0° |

---

## LED Patterns

The LED strip runs automatically — no driver input needed. Patterns are prioritized
top-to-bottom; the first matching condition wins.

> **Hardware status:** LEDs are not yet wired. The subsystem and patterns are
> implemented and ready. When physically connected, uncomment `ledSubsystem` in
> `RobotContainer` and wire the trigger bindings (see TODO comments in code).

### Disabled

| Pattern | Description |
|---------|-------------|
| **Teal / orange wave** (slow) | Team colors — on whenever the robot is disabled |

### Autonomous

| Pattern | Description |
|---------|-------------|
| **Orange / cyan wave** (fast) | Audience-visible "auto is running" indicator |

### Teleop (priority order)

| Priority | Condition | Pattern |
|----------|-----------|---------|
| 1 | Intaking | **Purple strobe** (fast) |
| 2 | Aim lock (shooting) | **Solid green** |
| 3 | Shift ending (≤ 5 s left) | **Orange strobe** (fast warning) |
| 4 | Shift active | **Green / black wave** (fast — "go score!") |
| 5 | Shift inactive | **Dim alliance color** (25% brightness) |

### Alliance Colors

| Source | Color |
|--------|-------|
| No FMS / unknown | Cyan |
| Blue alliance | Cyan |
| Red alliance | Red |

---

## Quick Summary

```
TURRET       → auto-tracks hub (shift active) or pass target (shift inactive)
FLYWHEEL     → auto pre-spins 5 s before each scoring window
RIGHT TRIG   → hub shot (shift-gated feed, auto-stops at boundary)
LEFT TRIG    → pass shot (ungated feed, tracks nearest pass target)
LEDs         → shift-aware patterns: green wave = go, dim = wait, orange strobe = hurry
```
