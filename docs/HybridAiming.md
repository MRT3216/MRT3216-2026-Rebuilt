# Hybrid Aiming — Control Scheme Reference

This document describes the two turret aiming modes and how to switch between them:

1. **Turret-Only** — The turret handles all aiming. The drivetrain is fully
   controlled by the driver.
2. **Hybrid (current default)** — The drivetrain auto-rotates the robot so the
   turret stays within its travel window (−90° to +130° by default). The turret
   corrects the residual error. Protects wiring from large turret swings.

---

## Quick Reference

| What to change | Turret-Only | Hybrid (current) |
|---|---|---|
| **Drive default command** | `DriveCommands.joystickDrive(...)` | `DriveCommands.joystickDriveAimAtTarget(...)` |
| **Shoot command** | `shooterSystem.aimAndShoot(...)` | `shooterSystem.hybridAimAndShoot(...)` |
| **Files to edit** | — | `RobotContainer.java` only |
| **Constants** | — | `ShooterConstants.HybridAimingConstants` |

---

## Step-by-Step: How to Switch Between Modes

All changes are in **`RobotContainer.java`**. No other files need editing.
Hybrid mode is **currently active**. The instructions below show how to switch
back to turret-only if needed, or serve as a reference for how it was wired.

### 1. Replace the drive default command

In `configureDefaultCommands()`, find the current drive default (~line 246):

```java
// CURRENT — turret-only
drive.setDefaultCommand(
        DriveCommands.joystickDrive(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX()));
```

Replace it with:

```java
// HYBRID — drivetrain assists with heading when target is outside turret comfort zone
drive.setDefaultCommand(
        DriveCommands.joystickDriveAimAtTarget(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX(),
                () -> AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint).toTranslation2d(),
                () -> drive.getPose(),
                () -> operatorController.getRightTriggerAxis() > 0.5)); // aim only while shooting
```

> **Key detail:** The `aimEnabled` supplier (7th argument) gates when the
> drivetrain heading assist is active. In the example above it's tied to the
> operator's right trigger — the drivetrain only auto-rotates while the
> operator is actively shooting. When the trigger is released, the driver has
> full manual control (identical to `joystickDrive`).
>
> All tuning constants (PID gains, deadband, home angle) are read from
> `ShooterConstants.HybridAimingConstants` — you only change them in one place.

### 2. Replace the shoot command (operator right trigger)

In `configureButtonBindings()`, find the hub-shot binding (~line 456):

```java
// CURRENT — turret-only
operatorController
        .rightTrigger()
        .whileTrue(
                shooterSystem.aimAndShoot(
                        () -> drive.getPose(),
                        () -> drive.getChassisSpeeds(),
                        () -> AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint),
                        3,
                        ShootingLookupTable.Mode.HUB,
                        () -> currentShootMode));
```

Replace it with:

```java
// HYBRID — turret clamped to travel window, drivetrain handles the rest
operatorController
        .rightTrigger()
        .whileTrue(
                shooterSystem.hybridAimAndShoot(
                        () -> drive.getPose(),
                        () -> drive.getChassisSpeeds(),
                        () -> AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint),
                        3,
                        ShootingLookupTable.Mode.HUB,
                        () -> currentShootMode));
```

> Notice the signature is now identical to `aimAndShoot` — making it a true
> drop-in replacement. The clamp and home angle are read from
> `HybridAimingConstants` internally.

### 3. (Optional) Replace the auto command

In the `NamedCommands` registration (~line 217), you can also swap the auto
shoot command if you want hybrid aiming in autonomous:

```java
NamedCommands.registerCommand(
        "Aim and Shoot",
        shooterSystem.hybridAimAndShoot(
                () -> drive.getPose(),
                () -> drive.getChassisSpeeds(),
                () -> AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint),
                3,
                ShootingLookupTable.Mode.HUB,
                () -> currentShootMode));
```

> **Note:** Auto paths would need to be aware that the robot will auto-rotate.
> This is usually fine for stationary shots but may affect path tracking during
> driving shots. Test carefully before using in competition auto routines.

### 4. Build and deploy

```
.\gradlew.bat deploy
```

---

## Step-by-Step: Switch Back to Turret-Only

Reverse the changes above:

1. Replace `joystickDriveAimAtTarget(...)` with `joystickDrive(...)` in the
   drive default command.
2. Replace `hybridAimAndShoot(...)` with `aimAndShoot(...)` in the operator
   right trigger binding (and auto command if changed). The argument list is
   identical — just change the method name.
3. Build and deploy.

No constants or other files need to change. The hybrid code stays in the
codebase but is simply not called.

---

## Tuning the Hybrid System

All constants are in `ShooterConstants.HybridAimingConstants`:

| Constant | Default | What it does |
|---|---|---|
| `kTurretHomeAngleDeg` | `0.0` | Turret resting direction. `0` = forward, `180` = rear-facing. |
| `kTurretMinDeg` | `-90.0` | Minimum turret travel (negative = left of home). Turret is clamped to `home + min`. |
| `kTurretMaxDeg` | `130.0` | Maximum turret travel (positive = right of home). Turret is clamped to `home + max`. |
| `kThresholdMarginDeg` | `15.0` | Width of the ramp zone (in degrees) on each side before the travel limit. Drivetrain correction fades in linearly over this margin. Set to `0.0` to disable the ramp and revert to hard on/off behavior. |
| `kHeadingKP` | `3.0` | How aggressively the drivetrain rotates toward the target. Increase if too sluggish. |
| `kHeadingKD` | `0.2` | Damping on the heading correction. Increase if the robot overshoots the target heading. |
| `kHeadingMaxVelocity` | `4.0 rad/s` | Speed limit on drivetrain rotation. Lower = smoother for the driver. |
| `kHeadingMaxAcceleration` | `8.0 rad/s²` | Acceleration limit. Lower = gentler ramp-up. |

### Graduated Drivetrain Assist (Ramp Zone)

Instead of a hard on/off boundary at the travel limit, the drivetrain heading
correction ramps in gradually on each side. This prevents a jarring snap when
the turret nears its clamp limit and gives the chassis a head start on rotating.

The travel window is **asymmetric** — the turret can swing further in one
direction than the other (e.g. −90° to +130° by default).

**Three zones** on each side (measured as the robot-relative angle from turret home):

```
 −90°           −75°                        +115°          +130°
  ├── Full corr ─┤──── Ramp zone ────────────┤── Ramp zone ─┤── Full corr ──►
    (100% assist)  (100% → 0%)  Turret only  (0% → 100%)    (100% assist)
                                (0% assist)
```

With the default constants (`kTurretMinDeg=-90`, `kTurretMaxDeg=130`, `kThresholdMarginDeg=15`):
- **−75° to +115°** — Inner zone: turret handles aiming alone, heading PID is reset.
- **−90° to −75°** (min side) — Ramp zone: drivetrain PID output is multiplied by
  a linear factor from 0.0 → 1.0 as the angle approaches −90°.
- **+115° to +130°** (max side) — Ramp zone: same, factor ramps 0.0 → 1.0 as
  the angle approaches +130°.
- **< −90° or > +130°** — Outer zone: full PID correction.

The 15° ramp margin on each side means the drivetrain reaches full correction
before the turret hits its mechanical limit — plenty of time for the chassis to
catch up.

### Telemetry keys (AdvantageScope / NetworkTables)

| Key | Type | Description |
|---|---|---|
| `HybridAiming/aimEnabled` | boolean | Is the heading controller active? |
| `HybridAiming/robotRelativeAngleDeg` | double | Angle from turret home direction to target. |
| `HybridAiming/outsideTravelWindow` | boolean | Is the target outside the turret travel window? |
| `HybridAiming/rampFactor` | double | Drivetrain assist strength: 0.0 = turret only, 1.0 = full correction. |
| `HybridAiming/headingOmega` | double | Heading correction output (rad/s), after ramp scaling. |
| `HybridAiming/rawTurretAzimuthDeg` | double | Unclamped turret angle from the shot solver. |
| `HybridAiming/clampedTurretAzimuthDeg` | double | Clamped turret angle actually sent to the turret. |
| `HybridAiming/turretClamped` | boolean | Was the turret angle clamped this loop? |

### Tuning procedure

1. Deploy with hybrid mode enabled.
2. Open AdvantageScope and watch `HybridAiming/*` keys.
3. Drive around the field with the shoot trigger held.
4. If the drivetrain correction feels too aggressive, lower `kHeadingKP` or
   `kHeadingMaxVelocity`.
5. If the drivetrain overshoots the heading, increase `kHeadingKD`.
6. If the turret is still hitting its limits, tighten the travel window
   (decrease `kTurretMaxDeg` or increase `kTurretMinDeg` toward 0) to make
   the drivetrain intervene sooner.
7. If `HybridAiming/turretClamped` is frequently `true`, the drivetrain isn't
   correcting fast enough — increase `kHeadingKP` or decrease the deadband.
8. Watch `HybridAiming/rampFactor` — it should transition smoothly from 0 → 1
   as the turret angle approaches the deadband. If the transition feels too
   abrupt, increase `kThresholdMarginDeg` (e.g. 15°). If too sluggish,
   decrease it (e.g. 5°).

---

## Architecture Overview

The hybrid system is two independent commands that work together:

```
┌─────────────────────────────────────────────────────────┐
│  DriveCommands.joystickDriveAimAtTarget                 │
│  (runs as drive default command)                        │
│                                                         │
│  • Driver controls translation (left stick)             │
│  • Driver controls rotation (right stick)               │
│  • When aimEnabled:                                     │
│    → Inner zone (min+margin to max−margin):             │
│      PID outputs zero, turret handles it                │
│    → Ramp zone (approaching min or max limit):          │
│      PID output scaled linearly 0% → 100%              │
│    → Outer zone (past min or max limit):                │
│      Full PID heading correction toward target          │
└─────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────┐
│  ShooterSystem.hybridAimAndShoot                        │
│  (runs while operator holds right trigger)              │
│                                                         │
│  • Identical to aimAndShoot except:                     │
│    → Turret azimuth CLAMPED to [home+min, home+max]     │
│    → FULL_STATIC mode locks turret at home angle        │
│  • Hood, flywheel, spindexer, kicker all unchanged      │
└─────────────────────────────────────────────────────────┘
```

The two commands share no state and communicate only through physics — the
drivetrain rotates the robot, which changes the robot-relative angle the shot
solver computes, which the turret then tracks (clamped).
