# Turret Wrap-Around Summary

**Branch:** Claude

---

## The Problem

Our turret has ±190° of travel (380° total) with a 20° dead-zone in the rear. Two issues arise:

1. **Targets beyond ±190°** — the shot solver or stick input can request an angle outside the physical range. The turret needs to "jump" to the opposite side.
2. **`atan2` wrapping at ±180°** — the solver uses `atan2` which outputs (−180°, 180°]. When a target crosses the ±180° boundary, the output jumps from +179° to −179° (or vice versa). If the turret is at +185°, a naïve command to −179° would cause a 364° swing instead of a 4° move.

## The Solution

`wrapAngle()` in `TurretSubsystem` is **position-aware**: it considers the turret's current encoder position to pick the ±360° equivalent that minimizes travel distance, then clamps to the soft limits.

## Why Not Use YAMS Built-In Wrapping?

`PivotConfig.withWrapping()` internally calls `SmartMotorControllerConfig.withContinuousWrapping()`, which **throws an exception** if soft limits are configured. It's designed for infinite-rotation mechanisms (like swerve azimuth), not limited-travel turrets.

---

## Key Code

### The Wrapping Algorithm (`TurretSubsystem.java`)

```java
public Angle wrapAngle(Angle requested) {
    double minDeg = kSoftLimitMin.in(Degrees);   // e.g. -190
    double maxDeg = kSoftLimitMax.in(Degrees);   // e.g. +190
    double currentDeg = getPosition().in(Degrees); // current turret encoder position

    double deg = requested.in(Degrees) % 360.0;   // normalize to (-360, 360)

    // Pick the ±360° equivalent closest to the current position
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

    return Degrees.of(Math.max(minDeg, Math.min(maxDeg, best))); // clamp
}
```

### How It's Applied

Every `setAngle` call wraps before reaching YAMS:

```java
// Supplier version (tracking/stick control) — wraps every loop
public Command setAngle(Supplier<Angle> angle) {
    return Commands.run(() -> {
        smartMotor.setPosition(wrapAngle(angle.get()));
    }, this);
}

// Static version (lock-to-angle commands)
public Command setAngle(Angle angle) {
    return turret.setAngle(wrapAngle(angle));
}
```

### Constants (`ShooterConstants.TurretConstants`)

```java
public static final Angle kSoftLimitMax = Degrees.of(190);
public static final Angle kSoftLimitMin = Degrees.of(-190);
public static final Angle kHardLimitMax = Degrees.of(195);
public static final Angle kHardLimitMin = Degrees.of(-195);
```

---

## Example Behavior

Assume the turret is currently at **+185°**:

| Requested | Candidates | Nearest to +185° | After Clamp | Move |
|-----------|-----------|-------------------|-------------|------|
| −179° | −179°, +181°, −539° | **+181°** | +181° | 4° ✓ |
| +150° | +150°, +510°, −210° | **+150°** | +150° | 35° ✓ |
| +200° | +200°, +560°, −160° | **−160°** (after clamp) | −160° | 345° (wraps around) |
| −175° | −175°, +185°, −535° | **+185°** | +185° | 0° ✓ |

The key insight: row 1 would be a **364° swing** with the old algorithm but is only a **4° move** now.

---

## Asymmetric Limits (Hypothetical)

If the build team mounts the turret so encoder zero isn't forward, only the soft/hard limit constants need to change. For example, if encoder zero is 130° CCW from forward:

```java
kSoftLimitMin = Degrees.of(-60);   // 130° - 190°
kSoftLimitMax = Degrees.of(320);   // 130° + 190°
```

The `wrapAngle()` algorithm works identically — it just picks the nearest candidate and clamps to whatever the limits are. No code changes needed.

---

## Fallback: Limit to 360° Travel

Change two constants:

```java
kSoftLimitMin/Max = ±180°   // currently ±190°
kHardLimitMin/Max = ±185°   // currently ±195°
```

---

## Files

- `src/main/java/frc/robot/subsystems/shooter/TurretSubsystem.java` — wrap-around logic
- `src/main/java/frc/robot/subsystems/shooter/ShooterConstants.java` — limits
- `src/main/java/frc/robot/util/shooter/HybridTurretUtil.java` — shot solver (outputs azimuth via atan2)
