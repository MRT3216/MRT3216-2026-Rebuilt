# Turret Aim Pipeline — Programmer Reference

**Branch:** Claude
**Updated:** 2025-03-21

---

## Overview

This document covers four topics programmers need to understand:

1. **Wrap-around** — how the turret handles angles beyond its physical travel range
2. **Encoder alignment** — how `kStartingPosition` maps the encoder to the real world
3. **Aim pipeline** — how the shot solver computes azimuth and how the turret follows it
4. **EasyCRT** — optional absolute-position bootstrapping using two absolute encoders and the Chinese Remainder Theorem

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

## 6. EasyCRT — Absolute Position at Boot

### The Problem

The turret uses a relative encoder that reads 0° at power-on. We rely on a human placing the turret on an alignment line and setting `kStartingPosition`. If someone forgets, or the turret gets bumped, the entire coordinate frame is wrong.

### The Solution: Chinese Remainder Theorem (CRT)

Two small-pinion absolute encoders mesh with the 90-tooth turret ring gear through different gear ratios. Because the pinion tooth counts are **coprime** (13T and 10T), the pair of fractional-rotation readings from the two encoders uniquely identifies the turret's position across the full travel range.

- **Encoder 1 (13T pinion):** REV Through Bore plugged into the SparkMax's absolute-encoder port. Read via `pivotMotor.getAbsoluteEncoder().getPosition()` — returns 0–1 rotations.
- **Encoder 2 (10T pinion):** Absolute encoder wired to RoboRIO DIO channel 0 as a PWM duty-cycle signal. Read via `turretPwmEncoder.get()` — returns 0–1 rotations.

This is implemented using YAMS's `EasyCRT` solver. At boot, it reads both encoders once, solves for the mechanism angle, and seeds the relative encoder — replacing the manual `kStartingPosition` entirely.

### How It Works (Math)

| Parameter | Value |
|-----------|-------|
| Ring gear teeth | 90T |
| Encoder 1 pinion (SparkMax) | 13T → 90/13 ≈ 6.923:1 ratio → wraps every 13/90 rot ≈ 52° |
| Encoder 2 pinion (PWM) | 10T → 9:1 ratio → wraps every 1/9 rot ≈ 40° |
| Coverage | lcm(13, 10) / 90 = 130/90 = **1.444 rotations** = 520° |
| Travel needed | ±190° = 380° = **1.056 rotations** |
| Margin | 520° − 380° = **140° spare** ✓ |

Since the coverage (520°) exceeds the travel range (380°), every position in the range maps to a unique pair of encoder readings. The CRT solver reconstructs the original position from these readings.

### Enabling CRT

CRT is controlled by a single boolean flag:

```java
// In ShooterConstants.TurretConstants:
public static final boolean kUseCRT = false;  // ← set true when encoders are installed & calibrated
```

When `kUseCRT = true`, the constructor:
1. Reads the SparkMax absolute encoder and the RoboRIO PWM encoder
2. Configures the EasyCRT solver with gear ratios, range, offsets, and inversions
3. Runs a single solve
4. If OK → seeds the relative encoder with the resolved angle
5. If FAILED → falls back to `kStartingPosition` and logs a warning

When `kUseCRT = false`, behavior is identical to before — `kStartingPosition` is used directly.

### Constants

All CRT constants live in `ShooterConstants.TurretConstants`:

```java
// Feature flag
public static final boolean kUseCRT = false;

// Gear geometry
public static final double kCRTCommonRatio = 1.0;       // ring gear is the mechanism gear
public static final int kCRTDriveGearTeeth = 90;         // ring gear
public static final int kCRTEncoder1PinionTeeth = 13;    // SparkMax abs enc (13T pinion)
public static final int kCRTEncoder2PinionTeeth = 10;    // RoboRIO PWM enc  (10T pinion)

// Mechanism range — derived from hard limits (changes automatically)
private static final double kCRTRangeMarginRot = 0.05;   // ≈ 18° extra each side
public static final Angle kCRTMechanismMin =
        Rotations.of(kHardLimitMin.in(Rotations) - kCRTRangeMarginRot);
public static final Angle kCRTMechanismMax =
        Rotations.of(kHardLimitMax.in(Rotations) + kCRTRangeMarginRot);

// Encoder offsets (calibrated per-robot — see verification section)
public static final Angle kCRTEncoder1Offset = Rotations.of(0.0);  // TODO: calibrate
public static final Angle kCRTEncoder2Offset = Rotations.of(0.0);  // TODO: calibrate

// Tolerance for the two encoder solutions to agree
public static final Angle kCRTMatchTolerance = Rotations.of(0.02); // ~7°

// Encoder sensing direction (set true if encoder reads backwards)
public static final boolean kCRTEncoder1Inverted = false;
public static final boolean kCRTEncoder2Inverted = false;
```

The PWM encoder DIO channel lives in `RobotMap.Shooter.Turret`:

```java
public static final int kAbsoluteEncoderPwmChannel = 0;  // RoboRIO DIO channel for 10T encoder
// The 13T encoder is on the SparkMax absolute-encoder port — no separate ID needed.
```

### Telemetry

At boot, the constructor logs to AdvantageKit:

| Key | Description |
|-----|-------------|
| `Shooter/Turret/CRT/Enabled` | Whether `kUseCRT` is true |
| `Shooter/Turret/CRT/Status` | `OK`, `NO_SOLUTION`, `AMBIGUOUS`, `INVALID_CONFIG`, or `NOT_ATTEMPTED` |
| `Shooter/Turret/CRT/ResolvedDeg` | The angle the solver produced (NaN if failed/disabled) |
| `Shooter/Turret/CRT/ErrorRot` | Internal error metric from the solver (rotations) |
| `Shooter/Turret/CRT/Iterations` | Number of iterations the solver used |

### Verification Process

Follow these steps when first installing CRT encoders on a new robot:

#### Step 1: Confirm Encoder Connectivity
1. Set `kUseCRT = false` (keep it disabled during wiring check)
2. For the **SparkMax encoder (13T):** Open REV Hardware Client, select the turret motor (CAN 54), and verify the absolute encoder tab shows a changing position when the turret is spun by hand
3. For the **PWM encoder (10T):** Open the DriverStation or Shuffleboard and check the DutyCycleEncoder widget on DIO 0 — it should show a changing value (0–1) when the turret rotates
4. Confirm both readings change **smoothly** (no jumps or dead spots)

#### Step 2: Determine Encoder Inversions
1. Spin the turret **CCW when viewed from above** (positive direction)
2. Watch each encoder's reported position
3. If an encoder's value **decreases** during CCW rotation, set its `kCRTEncoderXInverted = true`

#### Step 3: Calibrate Encoder Offsets
1. Physically position the turret at the **alignment line** (the known starting position)
2. Read each encoder's **absolute position** (in rotations):
   - SparkMax: REV Hardware Client → Absolute Encoder tab
   - PWM: Shuffleboard DutyCycleEncoder widget or DriverStation
3. Set `kCRTEncoder1Offset` and `kCRTEncoder2Offset` to **negate** those raw readings:
   - If encoder 1 reads +0.237 rot at the alignment line → `kCRTEncoder1Offset = Rotations.of(-0.237)`
   - If encoder 2 reads −0.412 rot → `kCRTEncoder2Offset = Rotations.of(0.412)`
4. This makes both encoders read 0 at the alignment line

#### Step 4: Enable and Test
1. Set `kUseCRT = true`
2. Position the turret at the alignment line, power on
3. Check AdvantageKit logs:
   - `CRT/Status` should be `OK`
   - `CRT/ResolvedDeg` should be close to `kStartingPosition` (within a few degrees)
   - `CRT/ErrorRot` should be very small (< 0.01)

#### Step 5: Full-Travel Validation
1. Power off, move the turret to ~5 different positions across the full range (e.g. −180°, −90°, 0°, +90°, +180°)
2. At each position:
   a. Power on the robot
   b. Record `CRT/ResolvedDeg` from the logs
   c. Independently verify the turret's angle with a protractor or known reference
   d. Confirm the values agree within ±5°
3. If any position gives `NO_SOLUTION` or `AMBIGUOUS`, check:
   - Encoder offsets (step 3)
   - Encoder inversions (step 2)
   - Mechanical mesh (are the pinions properly engaged with the ring gear?)

#### Step 6: Match-Day Confidence Check
Before each match, spot-check the boot log:
- `CRT/Status = OK`
- `CRT/ResolvedDeg` is reasonable for where the turret is visually pointing
- If status is not OK, the code automatically falls back to `kStartingPosition`

---

## Files

| File | What It Does |
|------|-------------|
| `src/.../shooter/TurretSubsystem.java` | `wrapAngle()`, `setAngle()`, encoder reading, CRT boot logic |
| `src/.../shooter/ShooterConstants.java` | All turret constants (limits, offsets, starting position, CRT config) |
| `src/.../constants/RobotMap.java` | CAN ID for turret motor, DIO channel for PWM CRT encoder |
| `src/.../util/shooter/HybridTurretUtil.java` | Shot solver — computes robot-relative azimuth via `atan2` |
| `src/.../systems/ShooterSystem.java` | Orchestrates solver → turret → hood → flywheel |
