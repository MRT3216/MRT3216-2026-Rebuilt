# PID / Feedforward / Motion Profile Tuning Guide

Recommended workflow for tuning every controlled mechanism on the robot. Work through each subsystem in the order listed — later subsystems depend on earlier ones being stable.

---

## Table of Contents

1. [General Principles](#general-principles)
2. [YAMS FF + Motion Profile Requirement](#yams-ff--motion-profile-requirement)
3. [Drivetrain (CTRE Swerve)](#1-drivetrain-ctre-swerve)
4. [Turret](#2-turret)
5. [Hood](#3-hood)
6. [Flywheel](#4-flywheel)
7. [Kicker](#5-kicker)
8. [Spindexer](#6-spindexer)
9. [Intake Rollers](#7-intake-rollers)
10. [Intake Pivot](#8-intake-pivot)
11. [PathPlanner](#9-pathplanner)
12. [Quick Reference Table](#quick-reference-table)

---

## General Principles

### Tuning Order (within any mechanism)

> *Based on [YAMS tuning presentation](https://yagsl.gitbook.io/yams/) methodology.*

**Step 0: Preparation**
- **Zero ALL gains** (kP, kI, kD, kS, kV, kA, kG = 0).
- **Limit motion profile velocity and acceleration** to safe values during tuning. You can always increase them later.
- Have AdvantageScope or Phoenix Tuner X open with a **Position vs Reference** graph (overlay actual position and setpoint on the same plot).

**For positional mechanisms with gravity (arms, pivots — turret, hood, intake pivot):**

1. **kG (gravity compensation):** Increase kG until the mechanism holds its position against gravity without drifting. YAMS applies `kG × cos(angle)` for arms — full compensation at horizontal, zero at vertical.
2. **kV (velocity FF):** Set a position setpoint and watch the Position vs Reference graph. **Match the slopes** of the position and reference lines during the constant-velocity portion of the motion profile. Start at 0.1, increase in large steps. When the slopes match, the mechanism cruises at the right speed.
3. **kA (acceleration FF):** Match the **acceleration portions** of the position vs reference graph (the curved parts at the start/end of moves). Start at 0.001, increase carefully. This shapes how quickly the mechanism ramps up/down.
4. **kP (proportional):** Increase kP until you see **slight overshoot**, then back off ~10-20%. kP handles the residual error that FF can't.
5. **kD (derivative):** Add only for high-speed or high-inertia mechanisms to dampen overshoot. "kD becomes your friend as speed and inertia increase."
6. **kI (integral):** Almost never needed. If you must use it, **clear the integral accumulator before setting kI to a nonzero value** — otherwise the accumulated error from before kI was active will cause a huge spike.

**For velocity mechanisms (flywheel, rollers, spindexer, kicker):**

1. **kS (static friction):** Slowly ramp voltage from 0 until the mechanism barely starts moving. That voltage is kS. *(Note: kS is not needed in simulation.)*
2. **kV (velocity FF):** Set a target velocity, adjust kV until steady-state velocity matches the setpoint. Verify at multiple speeds.
3. **kP:** Increase until the mechanism reaches setpoint within ~0.5s without oscillation.
4. **kD:** Only if there's oscillation. Usually 0 for velocity loops.
5. **kA (optional):** Only needed for faster spin-up/spin-down response.

> **Tip (from YAMS presentation):** When tuning gains other than kG, set the starting position/velocity, let the mechanism get close using existing gains, then temporarily set kP high to push it the rest of the way to setpoint. Then set kP back to 0 before continuing to tune the next FF gain. This lets you characterize FF at the correct operating point.

### Tools

| Tool | When to Use |
|------|-------------|
| **YAMS Live Tuning** | **Primary tuning tool for all YAMS mechanisms.** With `TelemetryVerbosity.HIGH`, every mechanism publishes editable NT entries for kP/kI/kD/kS/kV/kA/kG, setpoint (degrees or RPM), and motion profile limits (RPM, RPM/s). Change gains live without redeploying. |
| **AdvantageScope** | Log replay, overlaying setpoint vs measured. Use the [predefined swerve calibration layout](https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template/) for drive tuning. Import `AdvantageScope Swerve Calibration.json` from the swerve project folder. |
| **CTRE Tuner X** | Drive/steer gains, MotionMagic config, real-time signal plotting. |
| **REV Hardware Client** | SparkMax/Flex encoder verification, absolute encoder offset reads, firmware updates. |
| **WPILib SysId** | Automated FF characterization (quasistatic + dynamic). Already wired for drivetrain in `setupSysid()`. |
| **Elastic Dashboard** | Real-time telemetry graphs, boolean indicators ("at setpoint"), and YAMS tuning widgets side-by-side. |

### YAMS Live Tuning (how it works)

All YAMS mechanisms are configured with `TelemetryVerbosity.HIGH`. This publishes the following **editable** NetworkTables entries under each motor's telemetry table:

| NT Entry | Units | What it does |
|----------|-------|-------------|
| `closedloop/feedback/kP` | dimensionless | PID proportional gain — changes take effect immediately |
| `closedloop/feedback/kI` | dimensionless | PID integral gain |
| `closedloop/feedback/kD` | dimensionless | PID derivative gain |
| `closedloop/feedforward/kS` | volts | Static friction compensation |
| `closedloop/feedforward/kV` | volts/(unit/s) | Velocity feedforward |
| `closedloop/feedforward/kA` | volts/(unit/s²) | Acceleration feedforward |
| `closedloop/feedforward/kG` | volts | Gravity compensation (Arm FF only) |
| `closedloop/setpoint/position` | **degrees** | Tunable position setpoint |
| `closedloop/setpoint/velocity` | **RPM** | Tunable velocity setpoint |
| `closedloop/motionprofile/maxVelocity` | **RPM** | Motion profile cruise velocity |
| `closedloop/motionprofile/maxAcceleration` | **RPM/s** | Motion profile max acceleration |

> **Workflow:** Put the robot in **TEST MODE** (not teleop!) → Open Elastic or AdvantageScope → find the motor's tuning table → edit a gain → watch the response in real-time. No code changes, no redeploy. This is the fastest way to iterate.
>
> **⚠️ TEST MODE is required for YAMS Live Tuning.** The Live Tuning command is only available in test mode. In Driver Station, switch to "Test" mode before enabling. The robot will not drive in test mode — it only runs the tuning commands.
>
> **Live Tuning command path (Elastic):** Drag the Live Tuning button from `NT:/SmartDashboard/MECHANISM_NAME/Commands/SUBSYSTEM_NAME/Live Tuning` onto your dashboard. Enable the robot in test mode, then press the button to start the Live Tuning command. The command can be interrupted by any other trigger for that subsystem.
>
> **⚠️ Safety:** Always test live tuning in sim first before the real robot. Changing kP from 3 → 300 live could cause violent oscillation.
>
> **Telemetry units note:** Internally, YAMS PID operates on **rotations** (position) and **rotations per second** (velocity). However, the live tuning setpoint fields are published in **degrees** (position) and **RPM** (velocity) for human convenience. AdvantageScope can also convert display units via right-click → "Convert Units..." on any plotted field.
>
> **NT table locations:** All mechanism telemetry is under `NT:/Mechanisms`, not `NT:/SmartDashboard`. However `NT:/SmartDashboard` contains the Mechanism2d widgets and Live Tuning commands.

### AdvantageScope Tips

- **Setpoint vs measured overlay:** Drag both `*/angle` and `*/setpoint` (or `*/velocity` and `*/setpoint`) onto the same line graph. The gap tells you everything.
- **Unit conversion for graphs:** Right-click an axis → "Edit Axis" → multiplier of **360** converts rotations → degrees; multiplier of **60** converts RPS → RPM.
- **Discrete mode:** Toggle line graph to discrete mode for step response analysis — shows transitions more clearly.
- **Voltage overlay:** Plot voltage on a second Y-axis alongside position. If voltage saturates at ±12V, the motor can't give more. If it oscillates, kD is amplifying noise.
- **Timestamp notes:** Press `N` to mark each tuning trial ("kP=3.0", "kP=5.0") so you can compare runs later.
- **CSV export:** Select a time range → right-click → Export. Plot voltage vs velocity in a spreadsheet → slope = kV, y-intercept = kS. Quick SysId without the full tool.

### Elastic Dashboard Tips

- **Boolean "at setpoint" indicator:** We already log `Mechanisms/HoodIsMoving` etc. Add a boolean widget that turns green when `|setpoint - actual| < tolerance`.
- **Group tuning widgets:** Create a "Tuning" tab with the YAMS NT tuning entries for whichever mechanism you're working on — kP/kI/kD/kS/kV on the left, setpoint/angle graph on the right, current draw below.
- **Test setpoint selector:** Use `LoggedTunableNumber` widgets to quickly switch between test setpoints (e.g. "0°", "15°", "30°" for hood) without touching code.

### Phoenix Tuner X Tips (for CTRE mechanisms: drivetrain, hood, flywheel, intake rollers)

- **Position vs Reference overlay:** In Phoenix Tuner X, plot both `Position` and `ClosedLoopReference` signals on the same graph. The gap between them tells you how well FF+PID are tracking.
- **Group signals:** Select the motor → Signals → check `Position`, `ClosedLoopReference`, `ClosedLoopOutput`, `StatorCurrent` → plot together. This gives you a complete picture of what the controller is doing.
- **Sim connection:** Connect Phoenix Tuner X to `localhost` when running sim to get the same signal plotting on simulated motors.
- **kV slope matching:** With only kV active (kP=0), overlay Position and Reference. If the Position line's slope during the cruise phase is shallower than Reference, increase kV. Match the slopes.

### Safety Checklist (before every tuning session)

- [ ] Robot on blocks (wheels off ground) for drivetrain tuning
- [ ] Current limits set in constants (already done for all subsystems)
- [ ] Soft limits enabled in YAMS configs (prevents over-travel on pivots)
- [ ] Someone ready on the E-stop
- [ ] `tuningMode = true` in `Constants.java` (currently `true`)

---

## YAMS FF + Motion Profile Requirement

> **Key insight (verified in YAMS v2026.3.11 source + [YAMS Discord 3/16/2026](https://discord.gg/yams)):**
> For **positional** mechanisms (Pivot, Arm, Elevator), **kV and kA feedforward terms only do useful work when a motion profile is configured.**

### Why

Feedforward `kV` multiplies the *velocity setpoint* to compute the voltage needed to sustain that velocity. For positional control, the velocity setpoint comes from the motion profile — it's the speed the profile says the mechanism *should* be moving at right now. Without a profile there is no planned velocity, so:

- **TalonFX (MotionMagic):** The `Slot0.kV` and `Slot0.kA` values are applied against the MotionMagic-generated velocity/acceleration trajectory. With a plain `PositionDutyCycle` or `PositionVoltage` request (no profile), there's no velocity setpoint, so `kV` and `kA` multiply zero. Only `kS` (static friction) contributes.
- **SparkMax/Flex (MAXMotion):** Similarly, `closedLoop.feedForward.kV/kA` are used with `kMAXMotionPositionControl`. With plain `kPosition`, the Spark has no velocity reference.
- **RIO closed-loop (expo profile):** The `iterateClosedLoopController()` method uses the profile's `nextState.velocity` to compute `ff.calculateWithVelocities(current, next)`. Without a profile, it falls back to using the *actual measured velocity*, making the FF reactive instead of proactive.

### What this means for our robot

| Mechanism | Has FF? | Has Motion Profile? | FF Active? |
|-----------|---------|-------------------|------------|
| Turret | `kS=0, kV=1.0, kA=0.05` | ✅ 1000°/s, 7200°/s² | ✅ kV/kA fully used via MAXMotion |
| Hood | `kS=0.45, kV=3.0` | ✅ 270°/s, 270°/s² | ✅ kV fully used via MotionMagic |
| Intake Pivot | `kG=0.21, kS=0.11` (kV=0) | ✅ 90°/s, 90°/s² | ✅ kG/kS active; kV=0 is fine for now |

All positional mechanisms with FF already have motion profiles. ✅ No action needed.

### If you add FF to a positional mechanism in the future

**Always pair it with a motion profile.** Use `withClosedLoopController(kP, kI, kD, maxVelocity, maxAcceleration)` (the 5-argument form), not the 3-argument form.

> *"You have to have a motion profile for this to work. The kV and kA would only work if you have a velocity setpoint, which is given by motion profiles."* — nstrike (YAMS author), 3/16/2026

### Velocity mechanisms are unaffected

Flywheel, kicker, spindexer, and intake rollers use velocity control, where the *setpoint itself* is a velocity. The FF `kV` term multiplies the velocity setpoint directly — no profile needed. This is why FF-only control works well for flywheels.

---

## YAMS SysId Helpers

YAMS mechanism classes (`Arm`, `FlyWheel`, `Pivot`) have built-in `sysId()` methods that create a complete SysIdRoutine (quasistatic + dynamic, forward + reverse) with one call:

```java
// Example: run SysId on the intake pivot
public Command sysId() {
    return intakePivot.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));
}
```

### REV motors (SparkMax/SparkFlex)

YAMS uses duty cycle (not voltage control) internally to bypass the motor controller's internal voltage compensation, producing cleaner data. REVLib 2026+ also writes `.revlog` files automatically via Status Logger — these can be opened directly in AdvantageScope or converted to `.wpilog`.

### CTRE motors (TalonFX)

YAMS uses `VoltageOut` control requests and Signal Logger. You must manually start/stop Signal Logger:

```java
controller.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
controller.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));
```

Extract `.hoot` logs from the RoboRIO via Phoenix Tuner X → Tools → Extracting Signal Logs → export as WPILOG.

### When to use SysId vs manual tuning

| Approach | Best For | Our Mechanisms |
|----------|----------|----------------|
| **SysId** | Getting kS, kV, kA, kG to within ~5% | Intake pivot (kG), flywheel (kV), turret (kV) |
| **Manual** | Quick iteration when you already have a ballpark | Hood (already mostly tuned), kicker, spindexer |
| **YAMS Live Tuning** | Fine-tuning after SysId/manual gives starting values | All mechanisms |

---

## 1. Drivetrain (CTRE Swerve)

**File:** `TunerConstants.java` (**generated by Phoenix Tuner X — do not hand-edit gains here; use Tuner X to regenerate**)
**Motors:** 8× TalonFX (4 drive + 4 steer)
**Control:** Phoenix 6 Slot0 gains, Voltage output mode
**Swerve template:** [AdvantageKit TalonFX Swerve Template](https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template/)

> **AdvantageScope layout:** Import `AdvantageScope Swerve Calibration.json` from the swerve project folder (`File > Import Layout...`). It has predefined tabs for each tuning step below.

> ⚠️ **Note on `TunerConstants.java`:** This file is generated by Phoenix Tuner X. Motor gains (`steerGains`, `driveGains`), gear ratios, encoder offsets, and CAN IDs should only be changed by re-running the Tuner X generator. The `driveInitialConfigs` and `steerInitialConfigs` fields are the **user-customizable extension points** for things like current limits — those are safe to edit.

### Tuning Order (from AdvantageKit swerve template docs)

Follow this order — each step depends on the previous one:

1. **Steer PID** — modules must point accurately before drive characterization works
2. **Drive Feedforward (kS, kV)** — characterize with the "Drive Simple FF Characterization" auto
3. **Wheel Radius** — characterize with "Drive Wheel Radius Characterization" auto
4. **Drive PID (kP)** — tune after FF is set
5. **Max Speed Measurement** — measure actual top speed and update `kSpeedAt12Volts`
6. **Slip Current Measurement** — measure `kSlipCurrent` at which wheels break traction
7. **PathPlanner PID** — tune translation/rotation PID last, after drive is solid

### Step 1: Steer Motors (position control)

Current gains: `kP=100, kI=0, kD=0.5, kS=0.1, kV=2.48, kA=0`

**Workflow:**
1. Open Tuner X → select any steer motor → Configs → Slot 0.
2. **kS (static friction):** Command a very slow constant-velocity steer rotation. Increase kS from 0 until the module just barely starts moving. That voltage is kS.
3. **kV (velocity):** Command a moderate velocity and measure steady-state voltage. `kV = (voltage - kS) / velocity_rps`.
4. **kP:** Set a positional setpoint (e.g. 90°). Start kP at ~50, increase until the module reaches the target quickly without oscillation. You want <50ms settling.
5. **kD:** If there's overshoot, add kD in small increments (0.1–1.0). Current 0.5 is typical.
6. **kI:** Should remain 0 for steer. Steer doesn't need integral — the module homes every cycle.

> **AdvantageScope:** Plot `/RealOutputs/SwerveStates/Measured` vs `/RealOutputs/SwerveStates/SetpointsOptimized` to compare actual vs commanded module states.

### Step 2: Drive Feedforward Characterization (kS, kV)

Current gains: `kP=0.1, kI=0, kD=0, kS=0.26, kV=0.585`

> **Important:** AdvantageKit's template applies the swerve gear ratio using TalonFX firmware, not on the RIO. This means AdvantageKit FF gains are different from CTRE's default swerve code values.

**Preferred: "Drive Simple FF Characterization" auto** (quick, no SysId needed):
1. Uncomment `setupSysid()` in `RobotContainer.java` (line 224).
2. Place the robot in an open space.
3. Select the **"Drive Simple FF Characterization"** auto routine from the chooser.
4. Enable autonomous. The robot will slowly accelerate forward (quasistatic ramp).
5. Disable after ~5-10 seconds.
6. Check the **console output** for measured `kS` and `kV`. Copy them to `driveGains` in `TunerConstants.java` (via Tuner X regeneration or the driveGains config).

**Alternative: Full SysId** (also gets kA):
- The `setupSysid()` method also registers "Drive SysId (Quasistatic Forward/Reverse)" and "Drive SysId (Dynamic Forward/Reverse)" auto routines.
- Run all four, then analyze logs in the SysId tool. Export `.hoot` logs via Phoenix Tuner X → Tools → Extract Signal Logs → export as WPILOG, or use AdvantageKit logs directly (note: AK logs use radians, Phoenix uses rotations — convert appropriately).

### Step 3: Wheel Radius Characterization

Already wired as `DriveCommands.wheelRadiusCharacterization(drive)` in `setupSysid()`.

1. Place the robot **on carpet** (hard floor gives inaccurate results due to compression differences).
2. Select the **"Drive Wheel Radius Characterization"** auto routine.
3. Enable autonomous. The robot will slowly rotate in place.
4. Disable after at least **one full rotation**.
5. Check the console output for the measured wheel radius. Update `kWheelRadius` in `TunerConstants.java`.

> **Why this matters:** Effective wheel radius changes as wheels wear down, get swapped, or compress into carpet. Even small errors compound in odometry. Re-characterize regularly.

### Step 4: Drive PID Tuning (kP)

1. After FF is set, command a step velocity (e.g. 0 → 2 m/s).
2. In AdvantageScope, plot `/RealOutputs/SwerveStates/Measured` vs `/RealOutputs/SwerveStates/SetpointsOptimized`.
3. Increase kP until the rise time is <100ms without oscillation. 0.1 is a good starting point.
4. **kD:** Usually 0 for velocity loops — only add if you see ringing.

### Step 5: Max Speed Measurement

1. Set `kSpeedAt12Volts` in `TunerConstants.java` to the theoretical max speed (currently 6.02 m/s based on Kraken X60 free speed and 4.667:1 gearing).
2. Place the robot in an open space.
3. Plot `/RealOutputs/SwerveChassisSpeeds/Measured` in AdvantageScope.
4. In teleop, drive forward at full speed until velocity stops increasing.
5. Record the maximum velocity achieved and **update `kSpeedAt12Volts`** to this value. The effective max is typically slightly less than theoretical.

### Step 6: Slip Current Measurement

1. Place the robot against a **solid wall**.
2. In AdvantageScope, plot drive motor current (`/Drive/Module.../DriveCurrentAmps`) and drive velocity (`/Drive/Module.../DriveVelocityRadPerSec`).
3. Accelerate forward until the drive velocity suddenly increases (wheel slips). Note the current at that moment.
4. Update `kSlipCurrent` in `TunerConstants.java` to this value (currently 120A).

> **Drive stator current limit:** The drive motors currently have an 80A stator current limit in `driveInitialConfigs` for thermal protection. `kSlipCurrent` (120A) is a separate traction-detection mechanism. If you see brownouts during hard acceleration, lower the stator limit. If the robot feels sluggish, you can raise it (but not above `kSlipCurrent`).

---

## 2. Turret

**File:** `ShooterConstants.TurretConstants`
**Motor:** SparkMax + NEO
**Control:** YAMS Pivot (position + motion profile)
**FF type:** `SimpleMotorFeedforward(kS, kV, kA)`

Current gains: `kP=3.0, kI=0, kD=0, kS=0, kV=1.0, kA=0.05`
Motion profile: `1000 °/s max velocity, 7200 °/s² max accel` *(reduced from 1440/14400 — see warning below)*

> ⚠️ **Motion profile was exceeding motor capability.** NEO free speed (5676 RPM) through 27:1 gives a max mechanism speed of ~1261°/s. The original 1440°/s was unreachable. Now set to 1000°/s (~80% of theoretical max) with 7200°/s² accel for headroom.

**Workflow:** *(follows YAMS positional tuning order — see [General Principles](#tuning-order-within-any-mechanism))*
1. **kS:** The turret is horizontal, so no kG needed. Find the minimum voltage to get the turret moving from standstill. Currently 0 — may need a small value if the turret sticks at the gearbox.
2. **kV:** Set a position setpoint and overlay **Position vs Reference** in AdvantageScope or Phoenix Tuner X. **Match the slopes** of the two lines during the cruise phase of the motion profile. Start at 0.1, increase until the slopes match. Current value: 1.0.
3. **kA:** Match the **acceleration portions** (curved start/end of moves). Start at 0.001, increase. Current value: 0.05.
4. **kP:** Increase until you see slight overshoot, then back off. Command a step (e.g. 0° → 90°) and watch settling time — target <200ms. Current 3.0 is a reasonable starting point.
5. **kD:** Add if there's overshoot or oscillation around the setpoint. Start at 0.05, increase cautiously.
6. **Motion profile:** 1000°/s gives ~0.36s for a full 360° sweep. If the turret overshoots at high speed, reduce `kMaxVelocity`. If it's too slow for tracking, increase toward 1200°/s (but stay under the 1261°/s limit).

**Telemetry keys:** `Shooter/Turret/angle`, `Shooter/Turret/setpoint`

---

## 3. Hood

**File:** `ShooterConstants.HoodConstants`
**Motor:** TalonFX
**Control:** YAMS Pivot (position + motion profile via MotionMagic)
**FF type:** `SimpleMotorFeedforward(kS, kV, kA)`

Current gains: `kP=300, kI=0, kD=0, kS=0.45, kV=3.0, kA=0`
Motion profile: `270 °/s max velocity, 270 °/s² max accel`

**Workflow:** *(follows YAMS positional tuning order — see [General Principles](#tuning-order-within-any-mechanism))*
1. **kS:** The hood is small and light — 0.45 is already tuned. Verify by commanding a very slow sweep and checking that it moves smoothly from standstill.
2. **kV:** Overlay **Position vs Reference** in Phoenix Tuner X. Match the slopes during the cruise phase of the MotionMagic profile. Because the hood has a motion profile, kV is actively used by TalonFX MotionMagic to compute the voltage at each profile velocity setpoint. (See [YAMS FF + Motion Profile](#yams-ff--motion-profile-requirement) for why this matters.)
3. **kP:** The hood has a very high kP (300) because it's a small, low-inertia mechanism that needs to track LUT angles precisely. Verify by commanding step changes (e.g. 0° → 15° → 0°) and checking settling time < 100ms. Increase until slight overshoot, then back off.
4. **kD:** Add only if there's audible oscillation or chatter. The hood's low inertia means kD can cause noise amplification — be conservative.
5. **Motion profile:** 270°/s and 270°/s² is moderate. If the hood feels sluggish during rapid LUT angle changes, increase. If it overshoots, decrease.

**Telemetry keys:** `Hood/angle`, `Hood/setpoint`

---

## 4. Flywheel

**File:** `ShooterConstants.FlywheelConstants`
**Motors:** 2× TalonFX (leader + inverted follower)
**Control:** YAMS FlyWheel (velocity closed-loop)
**FF type:** `SimpleMotorFeedforward(kS, kV, kA)`

Current gains: `kP=0.2, kI=0, kD=0, kS=0.35, kV=0.12, kA=0`

**Workflow:** *(matches [YAMS flywheel tuning order](https://yagsl.gitbook.io/yams/documentation/details/integrations#example-tuning-a-flywheel))*
1. **kS:** Slowly ramp voltage from 0 until the flywheel just barely starts spinning. Record that voltage. *(Note: kS is not needed in simulation — sim motors have no static friction. Only tune kS on the real robot.)*
2. **kV:** Set a target velocity (with kP=0), then adjust kV until the steady-state velocity matches. Overlay **Velocity vs Reference** in Phoenix Tuner X — when the lines overlap at steady state, kV is correct. Verify at several speeds (1000, 2000, 3000, 4000 RPM).
3. **kP:** After FF is set, command a step velocity. Increase kP until the flywheel reaches setpoint within ~0.5s without oscillation. 0.2 is a good starting point.
4. **kD:** If there's oscillation, add derivative gain to dampen it. Usually 0 for velocity loops.
5. **kA (optional):** Only needed if you want faster spin-up. Command a step from 0 → 3000 RPM and measure the acceleration phase. Usually 0 is fine for flywheels.
6. **Tolerance:** `kVelocityTolerance = 30 RPM`. If shots are inconsistent, tighten this (costs spin-up time before feed starts).

**Telemetry keys:** `Flywheel/velocity`, `Flywheel/setpoint`
**Dashboard tunable:** `Shooter/FlywheelRPM`

---

## 5. Kicker

**File:** `ShooterConstants.KickerConstants`
**Motor:** SparkFlex + NEO Vortex
**Control:** YAMS FlyWheel (FF-only — PID intentionally zero)
**FF type:** `SimpleMotorFeedforward(kS, kV, kA)`

Current gains: `kP=0.0, kI=0, kD=0, kS=0.25, kV=0.12, kA=0`

**Workflow:**
1. The kicker only needs to spin at a consistent speed to feed balls — it doesn't need tight setpoint tracking.
2. **kS/kV:** Same procedure as flywheel. Command a few steady-state velocities, plot voltage vs velocity.
3. **PID:** Only add kP if you observe the kicker slowing down noticeably when a ball enters (load rejection). Otherwise, FF-only is fine.

**Dashboard tunable:** `Kicker/KickerRPM`

---

## 6. Spindexer

**File:** `ShooterConstants.SpindexerConstants`
**Motor:** SparkMax + NEO
**Control:** YAMS FlyWheel (velocity closed-loop)
**FF type:** `SimpleMotorFeedforward(kS, kV, kA)`

Current gains: `kP=0.02, kI=0, kD=0, kS=0.25, kV=0.6, kA=0`

**Workflow:**
1. Same as flywheel/kicker: characterize kS and kV at a few steady-state speeds.
2. **kP:** The spindexer feeds balls into the kicker — it needs moderate speed regulation but not tight tracking. 0.02 is very low; increase if balls feed inconsistently (the spindexer slows too much under load).
3. **Note:** The spindexer is in COAST idle mode. This is intentional — it should freewheel when not commanded.

**Dashboard tunable:** `Spindexer/IndexerRPM`

---

## 7. Intake Rollers

**File:** `IntakeConstants.Rollers`
**Motor:** TalonFX (Kraken X60 FOC)
**Control:** YAMS FlyWheel (velocity closed-loop)
**FF type:** `SimpleMotorFeedforward(kS, kV, kA)`

Current gains: `kP=0.5, kI=0, kD=0, kS=0.39, kV=0.24, kA=0`

**Workflow:**
1. Characterize kS/kV as with other velocity mechanisms.
2. **kP:** Intake rollers need decent load rejection (ball contact causes sudden load). 0.5 is already relatively aggressive — verify by running the intake and feeding balls through. If rollers slow down significantly on contact, increase kP. If they oscillate or buzz, decrease.

---

## 8. Intake Pivot

**File:** `IntakeConstants.Pivot`
**Motors:** 2× SparkFlex + NEO Vortex (left master, right follower inverted)
**Control:** YAMS Arm (position + motion profile)
**FF type:** `ArmFeedforward(kS, kG, kV)` — gravity-compensated

Current gains: `kP=0.0, kI=0, kD=0, kG=0.21, kS=0.11, kV=0, kA=0`
Motion profile: `90 °/s max velocity, 90 °/s² max accel`

> ⚠️ **PID is currently all zeros and feedforward is commented out in `IntakePivotSubsystem.java`.** All gains across the robot were set with limited testing time. Monday bring-up should re-tune FF and PID for every mechanism, starting with this one — uncomment `.withFeedforward(armFeedforward())`, tune kG first, then add PID.

**Workflow:** *(matches [YAMS arm tuning order](https://yagsl.gitbook.io/yams/documentation/details/integrations#example-tuning-an-arm))*
1. **kG (gravity compensation):** This is the critical first term. Hold the arm horizontal (0° from horizontal) and increase kG until it holds position against gravity. YAMS applies `kG × cos(angle)` internally — at vertical (±90° from horizontal) no gravity compensation is applied, at horizontal full kG is applied. 0.21 is the current value.
2. **kS/kV:** Run SysId (or manual characterization) to get these. Command a slow constant-velocity sweep, measure voltage and velocity. `kV = (voltage - kG×cos(θ) - kS) / velocity`. Currently kV=0 — this should be nonzero after tuning.
3. **kP:** After FF is stable, command a step (stow → deploy). Increase kP until the arm reaches the target within ~0.3s. Start at 1.0 and work up.
4. **kD:** Add to dampen overshoot. The intake arm has significant inertia (6.4 lbs, 11" long) — expect to need some kD (start at 0.05).
5. **Motion profile:** 90°/s and 90°/s² is conservative. Increase after PID is stable if you want faster deploy/stow.

**Telemetry keys:** `IntakePivot/angle`, `IntakePivot/setpoint`

---

## 9. PathPlanner

**File:** `Constants.PathPlannerConstants`
**Control:** WPILib PIDController (translation + rotation)

Current gains: `Translation: kP=5.0, kI=0, kD=0` | `Rotation: kP=5.0, kI=0, kD=0`

**Workflow:**
1. **Tune drivetrain first.** PathPlanner PID sits on top of the drive closed-loop — if drive gains are wrong, PP gains can't compensate.
2. **Translation kP:** Run a simple straight-line path. Watch the robot's actual path vs planned path in AdvantageScope. If the robot undershoots turns or drifts, increase kP. If it oscillates side-to-side, decrease.
3. **Rotation kP:** Run a path with heading changes. If the robot is slow to rotate to the target heading, increase. If it oscillates, decrease.
4. **kD:** Add if there's heading overshoot at the end of paths. Start at 0.1.
5. **kI:** Almost never needed. Only add if there's persistent steady-state heading error (suggests a gyro offset problem, not a gain problem).

---

## Quick Reference Table

| Mechanism | Motor(s) | Control | FF Terms | PID | Motion Profile | Constants File |
|-----------|----------|---------|----------|-----|---------------|----------------|
| Drive (steer) | TalonFX | Position | kS, kV | kP, kD | MotionMagic | `TunerConstants` |
| Drive (drive) | TalonFX | Velocity | kS, kV | kP | — | `TunerConstants` |
| Turret | NEO | Position+MP | kS, kV, kA | kP | 1000°/s, 7200°/s² | `ShooterConstants.TurretConstants` |
| Hood | TalonFX | Position+MP | kS, kV | kP | 270°/s, 270°/s² | `ShooterConstants.HoodConstants` |
| Flywheel | 2× TalonFX | Velocity | kS, kV | kP | — | `ShooterConstants.FlywheelConstants` |
| Kicker | Vortex | Velocity (FF-only) | kS, kV | — | — | `ShooterConstants.KickerConstants` |
| Spindexer | NEO | Velocity | kS, kV | kP | — | `ShooterConstants.SpindexerConstants` |
| Intake Rollers | Kraken X60 | Velocity | kS, kV | kP | — | `IntakeConstants.Rollers` |
| Intake Pivot | 2× Vortex | Position+MP | kG, kS, kV | ⚠️ kP=0 | 90°/s, 90°/s² | `IntakeConstants.Pivot` |
| PathPlanner | — | Translation+Rot | — | kP | — | `Constants.PathPlannerConstants` |

### Priority Order for Monday

> **All FF, PID, and motion profile gains were set with limited testing time. Plan to re-tune everything Monday.**

1. **Drivetrain** — everything else depends on accurate drive/odometry
2. **Turret** — needs to track accurately for shooting
3. **Flywheel** — RPM accuracy directly affects shot consistency
4. **Hood** — verify existing gains hold with the new LUT distances
5. **Intake Pivot** — uncomment FF, tune kG, then add PID
6. **Spindexer / Kicker / Rollers** — verify, adjust if feeding is inconsistent
7. **PathPlanner** — last, after drive is solid
