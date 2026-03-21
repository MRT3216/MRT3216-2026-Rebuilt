# Turret Aim Pipeline — Programmer Reference

**Branch:** Claude  
**Updated:** 2025-03-21

---

## Overview

This document covers three topics programmers need to understand:

1. **Wrap-around** — how the turret handles angles beyond its physical travel range
2. **Encoder alignment** — how `kStartingPosition` maps the encoder to the real world
3. **Aim pipeline** — how the shot solver computes azimuth and how the turret follows it

The turret's travel range is **not yet finalized** — build is designing the energy chain (wire carrier) that will determine the maximum rotation. The code is written so that **only constants change** when the range is decided; no logic changes are needed.

---

## 1. Wrap-Around

### The Problem

Two things conspire to create bad turret commands:

1. **Targets beyond the travel range** — the solver or stick input can request an angle outside the physical limits. The turret needs to "jump" to the reachable side.
2. **`atan2` wraps at ±180°** — the solver uses `atan2` which returns (−180°, 180°]. If the turret can travel past ±180° (e.g. ±190°), a target at +181° gets reported as −179°. If the turret is currently at +185°, a naïve command to −179° causes a **364° swing** instead of a 4° move.

### The Solution

`wrapAngle()` in `TurretSubsystem` is **position-aware** — it reads the turret's current encoder position, generates three ±360° candidates of the requested angle, picks the one closest to where the turret already is, then clamps to the soft limits.

```java
public Angle wrapAngle(Angle requested) {
    double minDeg = kSoftLimitMin.in(Degrees);
    double maxDeg = kSoftLimitMax.in(Degrees);
    double currentDeg = getPosition().in(Degrees);

    double deg = requested.in(Degrees) % 360.0;     // normalize to (-360, 360)

    double[] candidates = {deg, deg + 360.0, deg - 360.0};
    double best = candidates[0];
    double bestDist = Math.abs(candidates[0] - currentDeg);
    for (int i = 1; i < candidates.length; i++) {
        double dist = Math.abs(candidates[i] - currentDeg);
        if (dist < bestDist) {
            best = candidates[i];
            bestDist = dist;
        }
    }

    return Degrees.of(Math.max(minDeg, Math.min(maxDeg, best)));
}
```

Every `setAngle` call wraps before reaching YAMS:

```java
// Supplier version (tracking / stick control) — wraps every loop
public Command setAngle(Supplier<Angle> angle) {
    return Commands.run(() -> smartMotor.setPosition(wrapAngle(angle.get())), this);
}

// Static version (lock-to-angle commands)
public Command setAngle(Angle angle) {
    return turret.setAngle(wrapAngle(angle));
}
```

### Why Not Use YAMS Built-In Wrapping?

`PivotConfig.withWrapping()` calls `SmartMotorControllerConfig.withContinuousWrapping()`, which **throws an exception** if soft limits are also configured. It is designed for infinite-rotation mechanisms (swerve azimuth), not limited-travel turrets.

### Example (with ±190° limits)

Turret is currently at **+185°**:

| Requested | Candidates | Nearest to +185° | After Clamp | Actual Move |
|-----------|-----------|-------------------|-------------|-------------|
| −179° | −179°, +181°, −539° | **+181°** | +181° | 4° ✓ |
| +150° | +150°, +510°, −210° | **+150°** | +150° | 35° ✓ |
| +200° | +200°, +560°, −160° | **+200°** → clamp | +190° | 5° ✓ |
| −175° | −175°, +185°, −535° | **+185°** | +185° | 0° ✓ |

Row 1 is the critical case: old code would command a 364° swing; the position-aware version moves 4°.

---

## 2. Encoder Alignment & `kStartingPosition`

### The Problem

The turret uses a **relative encoder** (not absolute). At power-on the encoder reads 0°, regardless of where the turret is physically pointing. We always position the turret to an alignment line before turning on the robot, so boot position is repeatable — but that line might not correspond to "robot forward."

The shot solver computes azimuth in **robot-relative** coordinates where **0° = robot forward** (WPILib convention: +X forward, +Y left, CCW positive). If the encoder reads 0° at boot but the turret is actually pointing 76° left of forward, every shot will be off by 76°.

### The Fix: `kStartingPosition`

```java
public static final Angle kStartingPosition = Degrees.of(0);   // ← update when line is measured
```

YAMS's `PivotConfig.withStartingPosition(kStartingPosition)` calls through to the SparkMax wrapper which does:

```java
m_sparkRelativeEncoder.setPosition(startingPosition.in(Rotations));
```

This **seeds the encoder** at boot. If the alignment line is at 76° from forward, set `kStartingPosition = Degrees.of(76)`. Now:

- Encoder reads **76°** at boot (turret is physically at 76°) ✓
- Solver outputs **0°** meaning "point forward" → turret rotates from 76° to 0° on the encoder → physically forward ✓
- Solver outputs **30°** → encoder goes to 30° → physically 30° left of forward ✓

**The encoder's coordinate frame matches the solver's coordinate frame: 0° = robot forward.**

### What Needs To Be Measured

Once the build team provides the alignment line, someone needs to measure the angle from robot forward to the alignment line (in the turret's rotation plane, CCW positive when viewed from above). That angle becomes `kStartingPosition`.

- **Line is at robot forward:** `kStartingPosition = Degrees.of(0)` (current default)
- **Line is at 76° CCW from forward:** `kStartingPosition = Degrees.of(76)`
- **Line is at 45° CW from forward:** `kStartingPosition = Degrees.of(-45)`

---

## 3. Aim Pipeline End-to-End

Here is the full data flow from target to motor:

```
Field target (x, y, z)
        │
        ▼
HybridTurretUtil  ──────────────────────────────────────────────
│  1. Rotate kRobotToTurretTransform by robot heading to get   │
│     turret world-space origin (handles XY position offset)   │
│  2. Compute dx, dy from turret origin to target              │
│  3. azimuth = atan2(dy, dx) - robotHeading                   │
│     → robot-relative angle, 0° = forward, CCW positive       │
│  4. Return ShotSolution { turretAzimuth, hoodAngle, ... }    │
────────────────────────────────────────────────────────────────
        │
        ▼
ShooterSystem calls turret.setAngle(() -> solution.turretAzimuth())
        │
        ▼
TurretSubsystem.setAngle(Supplier<Angle>)
│  1. wrapAngle(requested):                                     │
│     - Generate ±360° candidates                               │
│     - Pick candidate nearest to current encoder position      │
│     - Clamp to [kSoftLimitMin, kSoftLimitMax]                 │
│  2. smartMotor.setPosition(wrapped)  → YAMS → SparkMax PID   │
─────────────────────────────────────────────────────────────────
```

### What kRobotToTurretTransform handles

The turret is not at the center of the robot. `kRobotToTurretTransform` is a **pure translation** (currently −5.5" X, +5.5" Y, +13.0625" Z) with **identity rotation**. The solver rotates this offset into field coordinates to get the turret's world position. This means shots are computed from the turret, not the robot center — correctly.

The rotation component is identity because the turret's encoder coordinate frame already aligns with the robot frame (once `kStartingPosition` is set correctly).

---

## 4. Adapting to Any Travel Range

The algorithm is generic. When build finalizes the energy chain, we only change constants:

### Symmetric limits (turret centered, equal travel both ways)

| Scenario | kSoftLimitMin | kSoftLimitMax | kHardLimitMin | kHardLimitMax | Dead Zone |
|----------|--------------|--------------|--------------|--------------|-----------|
| ±190° (current placeholder) | −190° | +190° | −195° | +195° | 20° rear |
| ±180° (exactly full rotation) | −180° | +180° | −185° | +185° | 0° (none) |
| ±150° (limited chain) | −150° | +150° | −155° | +155° | 60° rear |
| ±270° (generous chain) | −270° | +270° | −275° | +275° | 0°, full rear coverage |

> **Hard limits** are the physical stops enforced by YAMS in simulation. Set them 5° beyond the soft limits as a safety buffer. Soft limits are what `wrapAngle()` clamps to.

### Asymmetric limits (alignment line not centered in travel range)

If the alignment line is at a non-centered position, the soft limits simply shift. For example, if the turret can travel 190° in each direction from center, but the alignment line is 30° CW from the mechanical center:

```java
kStartingPosition = Degrees.of(-30);  // line is 30° CW from forward
kSoftLimitMin = Degrees.of(-220);     // -30° - 190°
kSoftLimitMax = Degrees.of(160);      // -30° + 190°
```

`wrapAngle()` works identically — it just picks the nearest candidate and clamps to whatever the limits are. **No code changes needed.**

### What happens with < 360° total travel?

If the total range is under 360° (e.g. ±150° = 300° total), there is a dead zone the turret cannot reach. `wrapAngle()` handles this gracefully: all ±360° candidates for an angle in the dead zone will fall outside the soft limits, and the clamp pushes the turret to the nearest reachable edge.

### What happens with ≥ 360° total travel?

If the total range is 360° or more (e.g. ±190° = 380° total), there is no dead zone — the turret can reach every direction. The `atan2` boundary at ±180° is still handled correctly by the position-aware candidate selection: the turret takes the short path instead of spinning the long way around.

---

## 5. Constants Summary

All turret geometry and limit constants live in `ShooterConstants.TurretConstants`:

```java
// Travel limits (TO BE UPDATED when energy chain is finalized)
public static final Angle kSoftLimitMax = Degrees.of(190);   // max encoder angle (wrapping clamp)
public static final Angle kSoftLimitMin = Degrees.of(-190);  // min encoder angle (wrapping clamp)
public static final Angle kHardLimitMax = Degrees.of(195);   // physical stop (sim safety)
public static final Angle kHardLimitMin = Degrees.of(-195);  // physical stop (sim safety)

// Encoder seed (TO BE UPDATED when alignment line is measured)
public static final Angle kStartingPosition = Degrees.of(0); // angle of alignment line from forward

// Turret position on robot (measured from CAD)
public static final Distance kTurretOffsetX = Inches.of(-5.5);    // behind center
public static final Distance kTurretOffsetY = Inches.of(5.5);     // left of center
public static final Distance kTurretOffsetZ = Inches.of(13.0625); // above floor
public static final Transform3d kRobotToTurretTransform = ...;    // built from above
```

---

## Files

| File | What It Does |
|------|-------------|
| `src/.../shooter/TurretSubsystem.java` | `wrapAngle()`, `setAngle()`, encoder reading |
| `src/.../shooter/ShooterConstants.java` | All turret constants (limits, offsets, starting position) |
| `src/.../util/shooter/HybridTurretUtil.java` | Shot solver — computes robot-relative azimuth via `atan2` |
| `src/.../systems/ShooterSystem.java` | Orchestrates solver → turret → hood → flywheel |
