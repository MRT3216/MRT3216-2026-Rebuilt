# Shooter Calibration Guide

This document describes how to use **Test Mode** (`testShoot`) to calibrate the
shooter's two-point flywheel model and populate the hood-angle lookup table (LUT).

---

## Prerequisites

| Item | Where |
|------|-------|
| `Constants.tuningMode` | Must be `true` (enables LoggedTunableNumbers on the dashboard) |
| Driver Station | Connected, robot enabled in **Teleop** with tuning mode active |
| Dashboard | AdvantageScope or Elastic — viewing `TestShoot/*` and `Tuning/*` keys |
| Balls | A supply of game pieces ready to feed |

---

## Dashboard Tunables

These values appear under `/Tuning/` in NetworkTables when `tuningMode` is active:

| Key | Default | Purpose |
|-----|---------|---------|
| `Shooter/FlywheelRPM` | 3000 | Flywheel RPM override. Leave at default for auto-RPM from the two-point model. Change to manually set RPM. |
| `Shooter/HoodAngleDeg` | 0 | Hood angle in degrees. Adjust until shots land in the hub. |

---

## NetworkTables Telemetry (logged while `testShoot` is active)

All keys are under `TestShoot/`:

| Key | Type | Description |
|-----|------|-------------|
| `distanceMeters` | double | Turret-to-hub distance in meters |
| `distanceInches` | double | Same distance in inches (convenience) |
| `effectiveRPM` | double | The RPM actually being commanded (auto or override) |
| `modelRPM` | double | What the two-point model would command at this distance |
| `rpmOverrideActive` | boolean | `true` when `Shooter/FlywheelRPM` differs from its default |
| `hoodAngleDeg` | double | Current hood angle tunable value |

---

## How `testShoot` Works

When the driver holds **Right Trigger** in test mode:

1. **Turret** locks at **0°** — point the robot directly at the hub.
2. **Hood** follows `Shooter/HoodAngleDeg` from the dashboard (live-adjustable).
3. **Flywheel RPM** is determined by:
   - **Auto mode** (default): the two-point linear model (`ShooterModel`) computes
     RPM from the turret-to-hub distance.
   - **Manual override**: if you change `Shooter/FlywheelRPM` on the dashboard away
     from its default (3000), the tunable value is used instead. The threshold is
     ±1 RPM from default.
4. **Feed**: clears the kicker briefly, then runs spindexer + kicker to feed balls.
5. **Telemetry**: distance, effective RPM, model RPM, and hood angle are logged
   every loop to `TestShoot/*`.

---

## Calibration Procedures

### Phase 1 — Build the Two-Point Flywheel Model

The flywheel model is a simple linear interpolation between two anchor points:
`(dMin, rpmAtMin)` and `(dMax, rpmAtMax)`.

#### Step 1: Closest Shot

1. Place the robot at the **closest** reasonable scoring distance from the hub.
2. Hold **Right Trigger** to start `testShoot`.
3. Read `TestShoot/distanceMeters` (or `distanceInches`) from the dashboard — this
   is your `dMin`.
4. Set `Shooter/HoodAngleDeg` to a reasonable starting angle (e.g. 10–15°).
5. Change `Shooter/FlywheelRPM` on the dashboard to override auto-RPM. Start low
   (e.g. 2000) and increase until balls consistently land in the hub.
6. Record the winning RPM as `kRpmAtMin` and the distance as `dMin`.

#### Step 2: Farthest Shot

1. Move the robot to the **farthest** reasonable scoring distance.
2. Repeat the same process — read the distance, dial in RPM.
3. Record as `kRpmAtMax` and `dMax`.

#### Step 3: Update Constants

Edit `ShooterConstants.ShooterModel`:

```java
public static final Distance dMin = Inches.of(YOUR_CLOSEST_DISTANCE_INCHES);
public static final Distance dMax = Inches.of(YOUR_FARTHEST_DISTANCE_INCHES);
public static final AngularVelocity kRpmAtMin = RPM.of(YOUR_CLOSEST_RPM);
public static final AngularVelocity kRpmAtMax = RPM.of(YOUR_FARTHEST_RPM);
```

#### Step 4: Verify

1. **Reset** `Shooter/FlywheelRPM` back to the default (3000) on the dashboard so
   auto-RPM kicks in from the model.
2. Move the robot to several intermediate distances and confirm
   `TestShoot/effectiveRPM` produces reasonable values.
3. Verify `TestShoot/rpmOverrideActive` shows `false`.

---

### Phase 2 — Populate the Hood-Angle LUT

With the two-point model in place, the flywheel RPM is now automatic. You need to
find the correct hood angle at multiple distances.

#### At Each Distance

1. Place the robot at a known distance.
2. Hold **Right Trigger**.
3. Leave `Shooter/FlywheelRPM` at default (auto-RPM from model).
4. Adjust `Shooter/HoodAngleDeg` until balls land in the hub.
5. Record from the dashboard:
   - `TestShoot/distanceMeters` (or inches)
   - `TestShoot/hoodAngleDeg`
6. Release trigger, move to the next distance, repeat.

#### Update the LUT

Add entries to `ShooterLookupTables.HUB`:

```java
new LookupRow(Meters.of(DISTANCE), Degrees.of(HOOD_ANGLE), Seconds.of(0)),
```

> **Note:** Time-of-flight (`Seconds.of(0)`) is a placeholder. It will be filled
> in later from video review of the shots in flight.

#### Recommended Distances

Sample at 5–8 distances spread across the scoring range, including the closest and
farthest points from Phase 1. More points near the extremes improve accuracy where
the relationship is most non-linear.

---

### Phase 3 — Time-of-Flight (future)

After recording video of shots at each LUT distance:

1. Review footage to measure ball flight time (release to score).
2. Update the `Seconds.of(...)` value in each `LookupRow`.
3. This improves the motion-compensation solver (`HybridTurretUtil`) which uses TOF
   to predict where to aim when the robot is moving.

---

## Test Mode Button Reference

| Button | Action |
|--------|--------|
| **A** (hold) | Aim turret at hub (turret tracking only, no shoot) |
| **B** (hold) | Run intake rollers at target velocity |
| **X** (hold) | Run spindexer feed |
| **Y** (hold) | Run kicker feed |
| **Left Bumper** (hold) | Deploy intake pivot briefly |
| **Right Bumper** (hold) | Agitate intake |
| **Right Trigger** (hold) | **testShoot** — full calibration routine |
| **Start** | Reset gyro to 0° |

---

## Troubleshooting

| Problem | Fix |
|---------|-----|
| `rpmOverrideActive` stuck on `true` | Reset `Shooter/FlywheelRPM` to 3000 on the dashboard |
| Distance reads 0 or NaN | Check that the robot pose is valid (gyro zeroed, vision connected) |
| Hood doesn't move | Verify `Shooter/HoodAngleDeg` is within 0–30° (soft limits) |
| Balls fly wildly | Confirm turret is at 0° and robot is pointed at the hub |
