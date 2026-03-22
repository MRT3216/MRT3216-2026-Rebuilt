# PID / Feedforward / Motion Profile Tuning Guide

Recommended workflow for tuning every controlled mechanism on the robot. Work through each subsystem in the order listed — later subsystems depend on earlier ones being stable.

---

## Table of Contents

1. [General Principles](#general-principles)
2. [YAMS Telemetry Verbosity Levels](#yams-telemetry-verbosity-levels)
3. [Top Team TunerConstants Comparison](#top-team-tunerconstants-comparison)
4. [TorqueFOC — Available Upgrade Path](#torquefoc--available-upgrade-path)
5. [YAMS FF + Motion Profile Requirement](#yams-ff--motion-profile-requirement)
6. [YAMS SysId Helpers](#yams-sysid-helpers)
7. [Drivetrain (CTRE Swerve)](#1-drivetrain-ctre-swerve) — _Physical Measurements, Steps 0-8: Pre-flight, Steer PID, Drive FF, Wheel Radius, Drive PID, Max Speed, Slip Current, Odometry, PathPlanner_
8. [Turret](#2-turret)
9. [Hood](#3-hood)
10. [Flywheel](#4-flywheel)
11. [Kicker](#5-kicker)
12. [Spindexer](#6-spindexer)
13. [Intake Rollers](#7-intake-rollers)
14. [Intake Pivot](#8-intake-pivot)
15. [PathPlanner](#9-pathplanner) _(redirects to Drivetrain Step 8)_
16. [Quick Reference Table](#quick-reference-table)

---

## General Principles

### Learning Resources

If you're new to PID tuning or want a refresher, work through these in order:

| Resource | What it Covers | Link |
|----------|---------------|------|
| **Mantik: PID Control** | PID theory (P/I/D terms), on-RIO vs on-controller, feedforward tuning process | [mantik.netlify.app/frc/frc-pid-control](https://mantik.netlify.app/frc/frc-pid-control) |
| **Mantik: PID Practice — Setup** | Setting up YAMS simulation, Elastic, and AdvantageScope for live tuning practice | [mantik.netlify.app/frc/pid-tuning-practice-setup](https://mantik.netlify.app/frc/pid-tuning-practice-setup) |
| **Mantik: PID Practice — Elevator** | Hands-on: binary-search kG, double-kP method, "rectangular position graph" goal | [mantik.netlify.app/frc/pid-tuning-practice-elevator](https://mantik.netlify.app/frc/pid-tuning-practice-elevator) |
| **Mantik: PID Practice — Arm** | Hands-on: precise kG (2-3 decimal places), kP tuning with overshoot, adding motion profiling | [mantik.netlify.app/frc/pid-tuning-practice-arm](https://mantik.netlify.app/frc/pid-tuning-practice-arm) |
| **Mantik: Trapezoidal Motion Profiling** | What trapezoidal profiles are, how to configure on REV/CTRE, tuning max velocity + acceleration | [mantik.netlify.app/frc/trapezoidal-motion-profiling](https://mantik.netlify.app/frc/trapezoidal-motion-profiling) |
| **Mantik: Exponential Motion Profiling** | Why exponential > trapezoidal for arms, kV/kA profile params, CTRE MotionMagicExpo + WPILib ExponentialProfile | [mantik.netlify.app/frc/exponential-motion-profiling](https://mantik.netlify.app/frc/exponential-motion-profiling) |
| **Mantik: AdvantageScope** | Data logging, 3D field visualization, swerve views, video sync, analysis workflow | [mantik.netlify.app/frc/advantagescope](https://mantik.netlify.app/frc/advantagescope) |
| **Mantik: Elastic Dashboard** | Real-time widgets, live tuning setup, field visualization, camera streams | [mantik.netlify.app/frc/elastic-basics](https://mantik.netlify.app/frc/elastic-basics) |
| **YAMS Tuning Presentation** | YAMS-specific tuning methodology, FF+MP requirement, telemetry verbosity | [yagsl.gitbook.io/yams](https://yagsl.gitbook.io/yams/) |
| **AK Swerve Template Docs** | Complete swerve calibration (FF, wheel radius, slip current, max speed, PathPlanner) | [docs.advantagekit.org/.../talonfx-swerve-template](https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template/) |

> **Recommended path for someone who hasn't tuned before:** Read the PID Control page → do the Elevator practice in sim → do the Arm practice in sim → then come back to this guide and tune the real robot. The sim practice builds intuition for what "good" vs "bad" tuning looks like.

### Tuning Order (within any mechanism)

> *Based on [YAMS tuning presentation](https://yagsl.gitbook.io/yams/) and [Mantik PID tuning methodology](https://mantik.netlify.app/frc/frc-pid-control) (adapted from SuperNURDS FRC 3255).*

**Step 0: Preparation**
- **Zero ALL gains** (kP, kI, kD, kS, kV, kA, kG = 0).
- **Limit motion profile velocity and acceleration** to safe values during tuning. You can always increase them later.
- Have AdvantageScope or Phoenix Tuner X open with a **Position vs Reference** graph (overlay actual position and setpoint on the same plot).

**For positional mechanisms with gravity (arms, pivots — turret, hood, intake pivot):**

1. **kG (gravity compensation):** Use the **binary search method** — start with a low value, double until the mechanism moves upward, then binary search between the last good value and the first bad value. Continue to **2-3 decimal places** for arms/pivots (they're finicky). The position line should be completely flat. *(YAMS applies `kG × cos(angle)` for arms — full compensation at horizontal, zero at vertical.)*
2. **kV (velocity FF):** Set a position setpoint and watch the Position vs Reference graph. **Match the slopes** of the position and reference lines during the constant-velocity portion of the motion profile. Start at 0.1, increase in large steps. When the slopes match, the mechanism cruises at the right speed.
3. **kA (acceleration FF):** Match the **acceleration portions** of the position vs reference graph (the curved parts at the start/end of moves). Start at 0.001, increase carefully. This shapes how quickly the mechanism ramps up/down.
4. **kP (proportional):** Use the **doubling method** — start at 0.1, double (0.2, 0.4, 0.8, 1.6...) until you see **slight overshoot**, then back off ~10-20%. The position graph should approach a **rectangular shape** — sharp rise, flat hold, sharp fall. If it looks like a slow curve, kP is too low. If it oscillates or overshoots, kP is too high.
5. **kD (derivative):** Add only for high-speed or high-inertia mechanisms to dampen overshoot. Start tiny (0.005-0.05). Too high = high-frequency jitter/noise causing motor heating. "kD becomes your friend as speed and inertia increase."
6. **kI (integral):** Almost never needed. If you must use it, **clear the integral accumulator before setting kI to a nonzero value** — otherwise the accumulated error from before kI was active will cause a huge spike.

**For velocity mechanisms (flywheel, rollers, spindexer, kicker):**

1. **kS (static friction):** Slowly ramp voltage from 0 until the mechanism barely starts moving. That voltage is kS. *(Note: kS is not needed in simulation.)*
2. **kV (velocity FF):** Set a target velocity, adjust kV until steady-state velocity matches the setpoint. Verify at multiple speeds.
3. **kP:** Increase until the mechanism reaches setpoint within ~0.5s without oscillation.
4. **kD:** Only if there's oscillation. Usually 0 for velocity loops.
5. **kA (optional):** Only needed for faster spin-up/spin-down response.

**What "well-tuned" looks like in the position graph:**
- ✅ **Rectangular/boxy** — sharp rise, flat top, sharp fall (like a square wave)
- ❌ **Slow curve** — kP too low, or FF not carrying its weight
- ❌ **Overshoot + oscillation** — kP too high, or motion profile constraints too loose
- ❌ **Creep/drift** — kG wrong (for gravity mechanisms) or kS wrong (for horizontal)

> **Tip (from YAMS presentation):** When tuning gains other than kG, set the starting position/velocity, let the mechanism get close using existing gains, then temporarily set kP high to push it the rest of the way to setpoint. Then set kP back to 0 before continuing to tune the next FF gain. This lets you characterize FF at the correct operating point.

### Motion Profiling: When and Which Type

Motion profiling is **required** for any positional mechanism to prevent overshoot and protect hardware. Without it, the mechanism jumps instantly to max output, causing jerky motion that can break chains, strip gears, or ram into hard stops.

| Profile Type | Where it runs | Best for | Our mechanisms |
|---|---|---|---|
| **Trapezoidal** (constant accel) | Motor controller firmware | Simple point-to-point moves, mechanisms without gravity | Turret (MAXMotion), Hood (MotionMagic) |
| **Exponential** (smooth accel curve) | Motor controller firmware (MotionMagicExpo) or RIO (WPILib ExponentialProfile) | Swerve azimuth, gravity-loaded arms where motor voltage-speed curve matters | Drive steer (MotionMagicExpo), Intake pivot (WPILib ExponentialProfile) |

**Tuning motion profile constraints** ([detailed guide](https://mantik.netlify.app/frc/trapezoidal-motion-profiling)):
1. **Max velocity first:** Set acceleration very high, increase velocity until the mechanism struggles. Back off 10-20%.
2. **Max acceleration second:** Keep velocity at max, increase acceleration until you see overshoot or vibration. Reduce by 15-25%.
3. **Balance:** If the position graph has overshoot at corners, reduce acceleration. If motion is sluggish, increase both proportionally.

> **Trapezoidal vs Exponential:** Trapezoidal profiles assume the motor can instantly reach any acceleration — which isn't true. Exponential profiles model the motor's voltage-speed curve, so acceleration naturally decreases as speed increases (more physically realistic). For swerve azimuth and gravity-loaded arms, exponential is preferred. For simple point-to-point moves (turret, hood), trapezoidal is fine. See [Mantik: Exponential Motion Profiling](https://mantik.netlify.app/frc/exponential-motion-profiling) for details.

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

> **Telemetry verbosity is tied to `tuningMode`:** When `tuningMode = true`, all YAMS mechanisms publish `TelemetryVerbosity.HIGH` (all tunable gains, setpoints, limits, temps, motion-profile params). When `tuningMode = false` (competition), verbosity drops to `MID` (voltage, current, rotor data — good for post-match analysis without the overhead of publishing every tunable gain each cycle). See [YAMS Telemetry Verbosity Levels](#yams-telemetry-verbosity-levels) for details.

---

## YAMS Telemetry Verbosity Levels

Controlled by `Constants.telemetryVerbosity()`, which returns `HIGH` when `tuningMode = true` and `MID` when `tuningMode = false`. The levels cascade — each higher level includes everything from the level below.

### LOW (minimal — match play)

Published fields per motor:
- **SetpointPosition** / **SetpointVelocity** — what the controller is targeting
- **MeasurementPosition** / **MeasurementVelocity** — raw sensor readings
- **MechanismPosition** / **MechanismVelocity** — after gear ratio conversion

This is enough to confirm mechanisms are reaching their setpoints. YAMS publishes setpoint position at this level, equivalent to CTRE's `getClosedLoopReference()` — **no additional reference logging code is needed.**

### MID (competition debugging)

Adds to LOW:
- **OutputVoltage** — voltage being applied to the motor
- **StatorCurrent** / **SupplyCurrent** — current draw (useful for post-match brownout analysis)
- **RotorPosition** / **RotorVelocity** — raw rotor data before gear ratio

This is the recommended level for competition. You get enough data to diagnose issues post-match (brownouts, stalls, mechanism failures) without the overhead of publishing every tunable gain.

### HIGH (tuning — full visibility)

Adds to MID:
- **All tunable gains:** kP, kI, kD, kS, kV, kG, kA (editable via NetworkTables)
- **All tunable setpoints:** TunableSetpointPosition, TunableSetpointVelocity
- **Motor temperature**
- **All config limits:** MechanismLowerLimit, MechanismUpperLimit, StatorCurrentLimit, SupplyCurrentLimit, MeasurementLowerLimit, MeasurementUpperLimit
- **Ramp rates:** OpenloopRampRate, ClosedloopRampRate
- **Motion profile params:** Published via tunable number entries
- **Boolean flags:** MechanismLowerLimit, MechanismUpperLimit, TemperatureLimit, VelocityControl, ElevatorFeedForward, ArmFeedForward, SimpleMotorFeedForward, MotionProfile, MotorInversion, EncoderInversion

> **Impact:** Each motor at HIGH publishes ~30+ NetworkTables entries per cycle. With 10+ motors, that's 300+ entries. At competition, switching to MID cuts this to ~10 entries per motor — a meaningful reduction in loop time and NT traffic.

### When to switch

| Situation | Verbosity | `tuningMode` |
|-----------|-----------|-------------|
| Shop tuning / practice | HIGH | `true` |
| Practice match (want full data) | HIGH | `true` |
| Qualification matches | MID | `false` |
| Elimination matches | MID | `false` |
| Post-match debugging (replay) | N/A — replay uses whatever was logged | — |

---

## Top Team TunerConstants Comparison

Cross-reference of our swerve constants against top FRC teams' public code. All teams below use TalonFX (Kraken) swerve with Phoenix 6.

### Drive Current Limits

| Team | Drive Stator | Drive Supply | Steer Stator | Slip Current | Notes |
|------|-------------|-------------|-------------|-------------|-------|
| **3216 (us)** | **80A** | — | **60A** | **120A** | Stator limit in `driveInitialConfigs` |
| 6328 (Mechanical Advantage) | — | — | 40A | 80A | Uses TorqueCurrentFOC (stator limit = slip current) |
| 254 (Cheesy Poofs) | — | 80A supply | 50A (disabled) | 80A | Supply limit (not stator); ramp periods 0.01s |
| 1678 (Citrus Circuits) | — | — | 60A | 80A | Default `driveInitialConfigs`; Tuner X generated |
| 2910 (Jack in the Bot) | — | — | 60A | 80A | (Standard CTRE template) |
| WaltonRobotics | — | — | 60A | 120A | Higher slip current |

**Key takeaways:**
- **80A drive stator/slip current is the standard.** 6328, 254, and 1678 all converge on 80A for drive traction limiting. Our 80A stator limit matches.
- **60A steer stator is standard** for Voltage mode. 6328 uses 40A because they use TorqueCurrentFOC where current is the control variable. Our 60A matches 1678 and WaltonRobotics.
- **254 uses supply current limiting** (80A supply) instead of stator. This is a different philosophy — supply limits protect the battery/PDP, while stator limits protect the motor and limit traction. Both approaches work.
- **Our 120A kSlipCurrent** needs verification. Most teams use 80A. Only WaltonRobotics matches our 120A. Run the slip current characterization test (Step 6 in [Drivetrain](#1-drivetrain-ctre-swerve)) to confirm the correct value for our robot's weight and tire compound.

### Drive/Steer Gains (Voltage Mode)

| Team | Steer kP | Steer kD | Steer kS | Steer kV | Drive kP | Drive kV |
|------|---------|---------|---------|---------|---------|---------|
| **3216 (us)** | **100** | **0.5** | **0.1** | **2.48** | **0.1** | **0.585** |
| 254 | 100 | 0.2 | 0 | 1.5 | 0.35 | 0.136 |
| 1678 | 100 | 0.5 | 0.1 | 1.16 | 0.1 | 0.124 |
| WaltonRobotics | 100 | 0.5 | 0.1 | 2.66 | 0.35 | 0.123 |

**Key takeaways:**
- **Steer kP=100 is universal.** Every team uses 100 for steer proportional gain in Voltage mode.
- **Steer kD ranges 0.2–0.5.** Our 0.5 matches 1678 and WaltonRobotics.
- **Steer kV varies significantly** (1.16–2.66) depending on gear ratio, motor characterization, and firmware version. Our 2.48 is on the higher end — may need re-characterization.
- **Drive kP is low** across the board (0.1–0.35). Our 0.1 matches 1678.
- **Drive kV varies** with gear ratio. Our 0.585 is higher than others — this is expected because AdvantageKit applies gear ratio differently (in firmware vs on RIO). Don't directly compare without accounting for this.

### Closed-Loop Output Type

| Team | Drive Output | Steer Output | Pro Licensed? |
|------|-------------|-------------|--------------|
| **3216 (us)** | **Voltage** | **Voltage** | **Yes** |
| 6328 | TorqueCurrentFOC | TorqueCurrentFOC | Yes |
| 254 | Voltage | Voltage | — |
| 1678 | Voltage | Voltage | — |
| WaltonRobotics | Voltage | Voltage | — |

**Voltage mode is the standard.** Only 6328 uses TorqueCurrentFOC. We have Phoenix Pro (confirmed by `FusedCANcoder` steer feedback) and the code is fully wired — switching is a one-line config change + full retune. See [TorqueFOC — Available Upgrade Path](#torquefoc--available-upgrade-path).

### Max Speed

| Team | kSpeedAt12Volts | Gear Ratio | Module |
|------|----------------|-----------|--------|
| **3216 (us)** | **6.02 m/s** (theoretical) | 4.667:1 | L2+ |
| 6328 | 4.69 m/s (measured) | — | — |
| 1678 | 5.91 m/s | 6.48:1 | — |
| WaltonRobotics | 4.64 m/s | 6.75:1 | — |

> **Action item:** Our 6.02 m/s is theoretical. Run the max speed measurement test (Step 5 in [Drivetrain](#1-drivetrain-ctre-swerve)) and update to the measured value.

---

## TorqueFOC — Available Upgrade Path

TorqueCurrentFOC (Torque Current Field-Oriented Control) is a Phoenix Pro feature where the motor controller's control variable is **stator current** (amps) rather than voltage. This provides more linear and predictable torque output, better traction control, and improved battery voltage independence.

> **We have Phoenix Pro.** `TunerConstants.java` already uses `SteerFeedbackType.FusedCANcoder` (line 57), which requires Phoenix Pro. TorqueFOC is available to us right now.

### Benefits of TorqueFOC

- **Battery-voltage independence:** Voltage mode output changes as battery sags (12V → 11V = 8% less torque). TorqueFOC commands current directly, so performance is consistent regardless of battery state.
- **More linear torque response:** Current is directly proportional to torque (torque = kT × current). Voltage has a nonlinear relationship with torque due to back-EMF.
- **Better traction control:** `kSlipCurrent` directly limits the torque applied to wheels, making slip behavior more predictable.
- **Simpler FF model:** With torque control, kS/kV/kA represent friction/back-EMF/inertia in current units, which map more directly to physics.

### What changes with TorqueFOC

- **Gain scales are completely different.** 6328's Voltage gains (steer kP=100) vs their TorqueFOC gains (steer kP=4000, turn kD=50) illustrate this — you **cannot** reuse Voltage gains.
- **kSlipCurrent becomes the peak current limit** for drive motors (since current is the control variable). The stator current limit in `driveInitialConfigs` still applies as a thermal backstop.
- **Full retune of every drive gain** is required — FF characterization, drive PID, steer PID, slip current, max speed. There's no shortcut.

### 6328's TorqueFOC Gains (reference)

| Parameter | 6328 TorqueFOC | Our Voltage | Notes |
|-----------|---------------|-------------|-------|
| Steer kP | **4000** | 100 | 40× larger — current units |
| Steer kD | **50** | 0.5 | 100× larger |
| Steer kS | **16** | 0.1 | Amps, not volts |
| Steer kV | **0** | 2.48 | Not needed — torque control handles back-EMF |
| Drive kP | **35** | 0.1 | 350× larger |
| Drive kS | **5** | 0.26 | Amps |
| Drive kV | **0** | 0.585 | Same reasoning |
| Slip current | **80A** | 120A | Peak torque = slip limit |
| TorqueClosedLoopRampPeriod | **0.02s** | — | Limits current slew rate |

### Our TorqueFOC readiness

The code is **fully wired** for TorqueFOC in `ModuleIOTalonFX.java`:
- `TorqueCurrentFOC` and `PositionTorqueCurrentFOC` requests are imported and instantiated (lines 16-19, 62-66)
- Peak torque current is configured from `kSlipCurrent` (lines 107-108)
- Switch statements select the request type based on `TunerConstants` output type (lines 230-265)

### How to enable TorqueFOC (step-by-step)

1. **Change output type** in `TunerConstants.java` (lines 41, 44):
   ```java
   private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.TorqueCurrentFOC;
   private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.TorqueCurrentFOC;
   ```
2. **Zero out all gains** in `steerGains` and `driveGains` — start from scratch.
3. **Re-run the entire drivetrain calibration** (Steps 1-7 in [Drivetrain](#1-drivetrain-ctre-swerve)). The AK characterization routines work with any output type — they will produce correct FF values for TorqueFOC.
4. **Consider adding `TorqueClosedLoopRampPeriod`** (0.02s) to `driveInitialConfigs` to prevent current spikes:
   ```java
   .withClosedLoopRamps(new ClosedLoopRampsConfigs().withTorqueClosedLoopRampPeriod(0.02))
   ```
5. **Re-measure slip current** (Step 6) — the value will differ because current is now the control variable.

### Recommendation

**Start on Voltage mode, consider TorqueFOC after initial tuning is complete.** Voltage mode is proven by 254, 1678, and the majority of top teams. Once the robot is driving well on Voltage, switching to TorqueFOC is a single-constant change + full retune. The benefits are most noticeable during:
- Late-match play when battery voltage sags
- Autonomous routines that require consistent acceleration
- Defense situations with high-current pushing

### Motion Profile Types by Mechanism

Each mechanism's motion profile was chosen for its specific use case:

| Mechanism | Profile Type | Where it runs | Why this profile |
|-----------|-------------|--------------|-----------------|
| Drive (steer) | **MotionMagicExpo** | TalonFX firmware | Exponential profile: smooth, continuous jerk. Ideal for azimuth that must track smoothly during driving. Uses `kV` and `kA` expo parameters, not trapezoidal cruise/accel. |
| Turret | **MAXMotion (trapezoidal)** | SparkMax firmware | Trap profile is appropriate for a turret that moves to discrete angles. The 1000°/s cruise / 7200°/s² accel gives fast, predictable moves. |
| Hood | **MotionMagic (trapezoidal)** | TalonFX firmware | Small mechanism, short moves. Trapezoidal is simple and works well. 270°/s cruise is conservative — can increase after tuning. |
| Intake Pivot | **ExponentialProfile** | RIO (WPILib) | Runs on the RIO because SparkFlex doesn't support exponential profiles natively. `ExponentialProfile` models motor voltage constraints more accurately than trapezoidal for arms with gravity — it accounts for the fact that a motor can accelerate faster going down than up. |

> **Trapezoidal vs Exponential:** Trapezoidal profiles assume constant max acceleration. Exponential profiles model the motor's voltage-speed curve, so acceleration naturally decreases as speed increases (more realistic). For swerve azimuth (MotionMagicExpo) and gravity-loaded arms (ExponentialProfile), exponential is preferred. For mechanisms with simple point-to-point moves (turret, hood), trapezoidal is fine.

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
**Control:** Phoenix 6 Slot0 gains, Voltage output mode (TorqueFOC available — see [TorqueFOC section](#torquefoc--available-upgrade-path))
**Swerve template:** [AdvantageKit TalonFX Swerve Template](https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template/)
**CAN bus:** CAN FD (`"CANFD"`) — odometry runs at **250 Hz** (see [Odometry Frequency](#odometry-frequency))

> **AdvantageScope layout:** Import `AdvantageScope Swerve Calibration.json` from the swerve project folder (`File > Import Layout...`). It has predefined tabs for each tuning step below.

> ⚠️ **Note on `TunerConstants.java`:** This file is generated by Phoenix Tuner X. Motor gains (`steerGains`, `driveGains`), gear ratios, encoder offsets, and CAN IDs should only be changed by re-running the Tuner X generator. The `driveInitialConfigs` and `steerInitialConfigs` fields are the **user-customizable extension points** for things like current limits — those are safe to edit.

> ⚠️ **AdvantageKit uses different gain scales than CTRE's default swerve code.** AK configures the TalonFX to apply the swerve gear ratio in firmware (via `Slot0.kV` in rotor-space). This means FF and PID gains from CTRE's example code or Phoenix Tuner X presets will **not** work directly — they must be re-characterized using the AK routines described below.

### Enabling Characterization Routines

All characterization autos are registered in `RobotContainer.setupSysid()` (line 550). To enable:

1. **Uncomment** `setupSysid();` in the `RobotContainer` constructor (line 224).
2. Deploy and reboot.
3. The dashboard auto chooser will now show characterization options alongside normal autos.
4. **Remember to re-comment** `setupSysid()` before competition — you don't want characterization routines in your auto chooser during matches.

Available routines:
- **"Drive Simple FF Characterization"** — quasistatic ramp, outputs kS and kV
- **"Drive Wheel Radius Characterization"** — rotates in place, outputs wheel radius
- **"Drive SysId (Quasistatic Forward/Reverse)"** — full SysId quasistatic tests
- **"Drive SysId (Dynamic Forward/Reverse)"** — full SysId dynamic tests

### Complete Calibration Order

Follow this order exactly — each step depends on the previous one:

| Step | What | Auto Routine? | Updates |
|------|------|:---:|---------|
| 0 | [Pre-flight checks](#step-0-pre-flight-checks) | — | Verify Tuner X, module offsets, CAN |
| 1 | [Steer PID](#step-1-steer-pid) | — | `steerGains` kS, kV, kP, kD |
| 2 | [Drive FF characterization](#step-2-drive-feedforward-characterization-ks-kv) | ✅ | `driveGains` kS, kV |
| 3 | [Wheel radius](#step-3-wheel-radius-characterization) | ✅ | `kWheelRadius` |
| 4 | [Drive PID](#step-4-drive-pid-tuning-kp) | — | `driveGains` kP |
| 5 | [Max speed measurement](#step-5-max-speed-measurement) | — | `kSpeedAt12Volts` |
| 6 | [Slip current measurement](#step-6-slip-current-measurement) | — | `kSlipCurrent` |
| 7 | [Odometry frequency](#step-7-odometry-frequency) | — | Verify `Drive.ODOMETRY_FREQUENCY` |
| 8 | [PathPlanner configuration](#step-8-pathplanner-configuration) | — | Mass, MOI, wheel COF, PID |

### Pre-Tuning Physical Measurements

Before any software calibration, take these physical measurements. Several calibration steps and PathPlanner depend on accurate values.

| Measurement | How | Where to Update | Current Value | Status |
|---|---|---|---|---|
| **Robot mass (with bumpers + battery)** | Bathroom scale or pit scale. Weigh complete competition-ready robot. | `Drive.java` → `ROBOT_MASS_KG`, `settings.json` → `"robotMass"`, `Constants.DriveConstants.kRobotMassKg` | **63.5 kg (140 lbs)** — build lead estimate, with bumpers + battery. | ✅ **Updated** (verify on scale) |
| **Bumper weight** | Weigh one bumper set on a scale | Used for mass total above | ~12 lbs (estimated) | ⚠️ **Needs measurement** |
| **Wheel radius (effective)** | Software characterization — see [Step 3](#step-3-wheel-radius-characterization) | `TunerConstants.java` → `kWheelRadius`, `settings.json` → `"driveWheelRadius"` | 1.8" / 0.04572 m (nominal) | ⚠️ **Needs characterization on carpet** |
| **Max speed** | Software measurement — see [Step 5](#step-5-max-speed-measurement) | `TunerConstants.java` → `kSpeedAt12Volts`, `settings.json` → `"maxDriveSpeed"` | 6.02 m/s (theoretical) | ⚠️ **Needs on-robot measurement** |
| **Slip current** | Software measurement — see [Step 6](#step-6-slip-current-measurement) | `TunerConstants.java` → `kSlipCurrent` | 120A (default) | ⚠️ **Needs wall test** |
| **Robot MOI** | PathPlanner GUI estimator, or empirical (run autos, adjust if rotation overshoots/undershoots) | `Drive.java` → `ROBOT_MOI`, `settings.json` → `"robotMOI"` | 6.883 kg⋅m² | ⚠️ **Verify empirically** |

> **When to take these:** Weigh the robot and bumpers at the first opportunity (pit scale at competition, or a bathroom scale at the shop). Everything else is done via software calibration routines.

> **Keep all three locations in sync.** After every measurement, update `Drive.java` (runtime), `settings.json` (PathPlanner GUI), and `Constants.DriveConstants` (documentation). Mismatched values mean PathPlanner generates paths the robot can't follow.

### Step 0: Pre-flight Checks

Before any tuning, verify the following:

- [ ] **Phoenix Tuner X connected.** Connect to the RoboRIO (USB or network). Verify all 8 TalonFX motors and 4 CANcoders show green in the device list.
- [ ] **Module offsets correct.** In Tuner X, select each CANcoder → Configs → Magnet Offset. With all wheels pointed straight forward, the offset should read ~0°. If any module is off by 90°/180°, the offset is wrong — re-run the Tuner X swerve generator.
- [ ] **Motor inversions correct.** Manually spin each wheel forward — the drive motor should report positive velocity. Manually rotate each azimuth CCW (top view) — the steer motor should report positive position. If not, fix inversions in Tuner X.
- [ ] **CAN bus healthy.** In Tuner X → CAN Diagnostics, verify 0% bus utilization error rate. All devices should be on the `CANFD` bus (CAN FD).
- [ ] **Firmware updated.** All TalonFX and CANcoder firmware should be the latest compatible with Phoenix 6 v26.
- [ ] **Robot on blocks** (wheels off the ground) for steer and initial drive tuning. Only remove blocks for Steps 3 (wheel radius), 5 (max speed), and 6 (slip current).

### Step 1: Steer PID

Current gains: `kP=100, kI=0, kD=0.5, kS=0.1, kV=2.48, kA=0`

The steer motors use **MotionMagicExpo** (exponential profile) in firmware. The gains work with this profile automatically.

**Workflow:**
1. Robot on blocks, deploy code.
2. **kS (static friction):** Command a very slow constant-velocity steer rotation. Increase kS from 0 until the module just barely starts moving. That voltage is kS. Current value: 0.1.
3. **kV (velocity feedforward):** Command a moderate angular velocity and measure steady-state voltage. `kV = (voltage - kS) / velocity_rps`. Current value: 2.48. Verify by overlaying position and reference — the slopes should match during the cruise phase.
4. **kP (proportional):** Set a positional setpoint (e.g. 0° → 90°). Start kP at ~50, increase until the module reaches the target quickly without oscillation. Target settling time: **<50ms**. Current value: 100 (standard across all top teams).
5. **kD (derivative):** If there's overshoot, add kD in small increments (0.1–1.0). Current 0.5 is typical (matches 1678, WaltonRobotics).
6. **kI:** Should remain **0** for steer. Steer doesn't need integral — the module homes every cycle.

**AdvantageScope verification:**
- Plot `/RealOutputs/SwerveStates/Measured` and `/RealOutputs/SwerveStates/SetpointsOptimized` on the same line graph.
- The measured angles should track the setpoints with minimal lag and no oscillation.
- Right-click the axis → Edit → multiplier of 360 converts rotations to degrees for easier reading.

### Step 2: Drive Feedforward Characterization (kS, kV)

Current gains: `kP=0.1, kI=0, kD=0, kS=0.25955, kV=0.58499`

> **Important: AdvantageKit gain scale.** AK's template configures the TalonFX to apply the swerve gear ratio in firmware, not on the RIO. This means AK's FF gains (`kS`, `kV`) are in rotor-space and will be numerically different from CTRE's default swerve template values. **Use the AK characterization routine — do not copy FF values from non-AK projects.**

**Preferred method: "Drive Simple FF Characterization" auto routine** (quick, no SysId tool needed):

1. Place the robot in an **open space** (it will drive forward in a straight line). Wheels on the ground.
2. Select **"Drive Simple FF Characterization"** from the dashboard auto chooser.
3. Enable autonomous mode. The robot will slowly ramp up speed (quasistatic acceleration forward).
4. Let it run for **5–10 seconds**, then disable.
5. Check the **Driver Station console output** — it will print the measured `kS` and `kV` values.
6. Copy these values to `driveGains` in `TunerConstants.java`:
   ```java
   private static final Slot0Configs driveGains =
           new Slot0Configs().withKP(0.1).withKI(0).withKD(0).withKS(/* measured */).withKV(/* measured */);
   ```
7. **Repeat 2–3 times** and average the results for better accuracy.

> **Sanity check:** For Kraken X60 with 4.667:1 gearing and AK firmware-applied ratio, expect kV in the range of 0.4–0.7 and kS in the range of 0.1–0.4. Values far outside this range indicate a problem (wrong gear ratio, worn wheels, incorrect unit conversion).

**Alternative method: Full SysId** (also gets kA for acceleration feedforward):

The `setupSysid()` method registers four additional routines:
- "Drive SysId (Quasistatic Forward)" — slow ramp, forward
- "Drive SysId (Quasistatic Reverse)" — slow ramp, reverse
- "Drive SysId (Dynamic Forward)" — step voltage, forward
- "Drive SysId (Dynamic Reverse)" — step voltage, reverse

Run all four, then analyze using the WPILib SysId tool. To extract logs:
- **AdvantageKit logs:** Already in WPILOG format in the `logs/` directory. **Note:** AK logs positions in radians, not rotations — account for this when loading into SysId.
- **Phoenix Signal Logs:** Extract `.hoot` files via Phoenix Tuner X → Tools → Extract Signal Logs → Export as WPILOG for SysId analysis.

### Step 3: Wheel Radius Characterization

Current value: `kWheelRadius = Inches.of(1.8)` (1.8 inches = 0.04572 m)

Already wired as `DriveCommands.wheelRadiusCharacterization(drive)` in `setupSysid()`.

**Procedure:**

1. Place the robot **on carpet** — not on hard floor. Wheel compression into carpet changes the effective radius, and you want to characterize the radius as it will be during matches.
2. Select **"Drive Wheel Radius Characterization"** from the dashboard auto chooser.
3. Enable autonomous. The robot will slowly **rotate in place** (all modules point tangentially).
4. Let it run for **at least one full rotation** (preferably 2–3 rotations for better accuracy). The slower and more rotations, the more accurate.
5. Disable.
6. Check the **Driver Station console output** for the measured wheel radius.
7. Update `kWheelRadius` in `TunerConstants.java`:
   ```java
   public static final Distance kWheelRadius = Inches.of(/* measured value */);
   ```

> **How it works:** The routine uses the gyro's measured rotation and the drive encoders' measured distance to compute `radius = distance_driven / (θ_gyro × gear_ratio)`. The gyro provides an absolute ground-truth rotation angle.

> **When to re-characterize:** Re-run this whenever you:
> - Swap wheels (new tread compound or worn treads)
> - Move to a different carpet surface (practice vs competition)
> - See odometry drift in PathPlanner autos (wheels report different distance than actual)

> **Expected range:** For our 3.6" diameter wheels, expect a value of ~1.70–1.85 inches depending on compression. The theoretical 1.8" is a starting point.

### Step 4: Drive PID Tuning (kP)

After FF is set and wheel radius is accurate:

1. Deploy code with the new FF values.
2. In teleop, command a velocity step — release the joystick, then push forward abruptly (0 → ~2 m/s).
3. In AdvantageScope, plot `/RealOutputs/SwerveStates/Measured` vs `/RealOutputs/SwerveStates/SetpointsOptimized`.
4. **Look at the velocity traces.** With only FF (kP=0), the measured velocity should roughly follow the setpoint but may lag or undershoot slightly.
5. Increase kP until the **rise time** is <100ms and there is no steady-state error. Start at 0.05, increase to 0.1, 0.2, etc.
6. **kD:** Usually **0** for velocity loops — only add if you see ringing/oscillation. kD amplifies encoder noise, which is bad for velocity control.
7. Current kP=0.1 is a reasonable starting point (matches 1678).

> **If oscillating:** Reduce kP. If there's steady-state velocity error even with kP, it's an FF problem (kV too low or too high), not a PID problem.

### Step 5: Max Speed Measurement

Current value: `kSpeedAt12Volts = MetersPerSecond.of(6.022849)` (theoretical)

**Why this matters:** `kSpeedAt12Volts` is used to normalize joystick inputs — a value of 1.0 on the joystick commands `kSpeedAt12Volts` m/s. If this value is higher than the actual max speed, the robot can never reach full speed and the FF will be slightly off. If it's lower, the joystick saturates before full deflection.

**Procedure:**

1. Place the robot in an **open space** (at least 30 feet of straight driving room). Fully charged battery.
2. Set `kSpeedAt12Volts` to the **theoretical** value (currently 6.022849 m/s) if not already.
3. Deploy code.
4. In AdvantageScope, plot **`/RealOutputs/SwerveChassisSpeeds/Measured`** (look at the `vx` component — forward velocity).
5. In teleop, drive **full speed forward** on a straight line. Hold full joystick deflection for at least 3 seconds until velocity stabilizes.
6. Record the **peak sustained velocity** (not the momentary spike, the stable plateau).
7. Update `kSpeedAt12Volts` to this measured value:
   ```java
   public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(/* measured peak velocity */);
   ```

> **Expected:** For our Kraken X60 with 4.667:1 gearing, expect ~5.5–5.9 m/s measured (vs 6.02 m/s theoretical). The gap is due to friction, motor efficiency at load, and battery voltage sag.

> **Also update PathPlanner settings.json:** `"maxDriveSpeed"` should match your measured max speed. Currently set to 8.0 m/s, which is unrealistically high.

### Step 6: Slip Current Measurement

Current value: `kSlipCurrent = Amps.of(120)`

`kSlipCurrent` defines the stator current at which the drive wheels lose traction and start slipping. This value is used for traction control — the robot limits acceleration to keep current below this threshold.

**Procedure:**

1. Place the robot against a **solid, immovable wall** (e.g., a concrete wall or heavy workbench). All four wheels on carpet.
2. In AdvantageScope, plot two signals on the same graph (use two Y-axes):
   - **Drive current:** `/Drive/Module0/DriveCurrentAmps` (or any module index)
   - **Drive velocity:** `/Drive/Module0/DriveVelocityRadPerSec`
3. In teleop, **slowly** accelerate forward into the wall (gradually push the joystick from 0% → 100%).
4. Watch the velocity signal. Initially velocity will be ~0 (wheels grip, robot is against wall). At some current level, velocity will **suddenly jump up** — that's the wheel slipping.
5. Note the **current at the moment velocity jumps**. That's your slip current.
6. Update `kSlipCurrent` in `TunerConstants.java`:
   ```java
   private static final Current kSlipCurrent = Amps.of(/* measured value */);
   ```
7. **Repeat 2–3 times** with different modules and average.

> **Expected range:** For Kraken X60 on competition carpet with standard tread, most teams see 60–100A. 6328, 254, and 1678 all use 80A. Our current 120A may be higher than actual — verify with this test.

> **Relationship to stator current limit:** The drive motors have an 80A stator current limit in `driveInitialConfigs` for thermal protection. `kSlipCurrent` is used by the swerve kinematics for traction allocation. If `kSlipCurrent` < stator current limit, the stator limit is never the binding constraint (good). If `kSlipCurrent` > stator limit (like our current 120A > 80A), the stator limit prevents the robot from ever reaching slip — which means traction control isn't doing anything useful. **After measuring, ensure `kSlipCurrent` is set to the actual slip point.**

### Step 7: Odometry Frequency

Configured automatically in `Drive.java` (line 55):
```java
static final double ODOMETRY_FREQUENCY = TunerConstants.kCANBus.isNetworkFD() ? 250.0 : 100.0;
```

Since our CAN bus is **CAN FD** (`"CANFD"` in `TunerConstants.java`), odometry runs at **250 Hz** (4ms period). This is the AdvantageKit recommendation for CAN FD buses.

**What this controls:**
- The `PhoenixOdometryThread` samples drive/steer positions and the Pigeon 2 yaw at 250 Hz.
- These high-frequency samples are batched and processed in the main 50 Hz control loop.
- Higher frequency = better odometry accuracy during fast maneuvers.

**When to change:**
- **Standard CAN (not FD):** Automatically drops to 100 Hz. No action needed.
- **CPU overload:** If loop overruns are frequent, you can reduce to 200 Hz, but this is rarely necessary.
- **Multiple high-frequency buses:** If you have other high-frequency CAN traffic, reducing odometry frequency can help.

> **No action needed for our setup.** CAN FD at 250 Hz is correct and optimal.

### Step 8: PathPlanner Configuration

PathPlanner requires accurate robot physical parameters for path following. These are configured in two places that **must stay in sync**:

**1. Runtime config** (`Drive.java` lines 66-80) — used by the robot at runtime:
```
ROBOT_MASS_KG = 63.503  ← ✅ Updated (140 lbs / 63.5 kg — build lead estimate with bumpers + battery)
ROBOT_MOI = 6.883       (kg⋅m² — verify empirically with auto paths)
WHEEL_COF = 1.2          (rubber on carpet — reasonable)
```

**2. PathPlanner GUI settings** (`src/main/deploy/pathplanner/settings.json`) — used by PathPlanner desktop app for path generation:
```
"robotMass": 63.503      ← ✅ Updated (140 lbs with bumpers + battery)
"robotMOI": 6.883        ← ✅ Matches Drive.java
"wheelCOF": 1.2          ← ✅ Updated (was 2.255 — unrealistic)
"driveWheelRadius": 0.04572  ← ✅ Updated (matches kWheelRadius, re-characterize in Step 3)
"driveGearing": 4.66666667   ← ✅ Matches
"maxDriveSpeed": 6.022849    ← ✅ Updated (was 8.0 — unreachable. Re-measure in Step 5)
"driveCurrentLimit": 80.0    ← ✅ Updated (was 60A — matches driveInitialConfigs stator limit)
```

> ⚠️ **Remaining action:** Verify robot mass on a scale when possible. Current value (63.5 kg / 140 lbs) is a build lead estimate — confirm with a pit scale at the next event. Update `Drive.java` `ROBOT_MASS_KG`, `settings.json` `"robotMass"`, and `Constants.DriveConstants.kRobotMassKg` if the measured value differs.

**PathPlanner PID tuning** (`Constants.PathPlannerConstants`):

Current gains: `Translation: kP=5.0, kI=0, kD=0` | `Rotation: kP=5.0, kI=0, kD=0`

1. **Tune drivetrain first.** PathPlanner PID sits on top of the drive closed-loop — if drive gains are wrong, PP gains can't compensate.
2. **Translation kP:** Run a simple straight-line path. Watch the robot's actual path vs planned path in AdvantageScope. If the robot undershoots turns or drifts, increase kP. If it oscillates side-to-side, decrease.
3. **Rotation kP:** Run a path with heading changes. If the robot is slow to rotate to the target heading, increase. If it oscillates around the heading, decrease.
4. **kD:** Add if there's heading overshoot at the end of paths. Start at 0.1.
5. **kI:** Almost never needed. Only add if there's persistent steady-state heading error (suggests a gyro offset problem, not a gain problem).

**Estimating MOI:**
- The simplest method: use PathPlanner's built-in MOI estimator in the GUI.
- Alternatively: `MOI ≈ (1/12) × mass × (length² + width²)` for a uniform rectangular robot.
- For our robot: `(1/12) × 63.5 × (0.876² + 0.883²) ≈ 8.2 kg⋅m²`. Our current 6.883 is lower, suggesting the mass is concentrated closer to the center (which is typical with a heavy center turret). **Verify by running auto paths** — if the robot overshoots rotations, MOI is too low; if it's sluggish to rotate, MOI is too high.

### Advanced: Profiled Turning PID (MotionMagicExpo)

Our steer motors already use MotionMagicExpo (exponential profile) in firmware. This is configured in the AK swerve template's `ModuleIOTalonFX` constructor where it sets up MotionMagic expo parameters.

MotionMagicExpo uses `kV` and `kA` from the MotionMagic config (not the same as Slot0 `kV`/`kA`) to generate smooth exponential motion profiles. The profile naturally accounts for the motor's voltage-speed curve, producing smoother azimuth transitions than trapezoidal profiles.

**If considering TorqueFOC for steer:** Replace the `MotionMagicVoltage` request with `MotionMagicTorqueCurrentFOC`. The MotionMagic parameters stay in the same config — only the control request type and Slot0 gains change.

### Advanced: Swerve Setpoint Generator (254's Algorithm)

PathPlanner includes an implementation of 254's swerve setpoint generator algorithm. This optimizes module states to respect kinematic constraints (max module speed, max module acceleration) while minimizing scrub.

**Status:** Not currently enabled in our code. This is an advanced optimization — only consider after all basic calibration is complete and PathPlanner paths are tracking well.

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

See [Step 8: PathPlanner Configuration](#step-8-pathplanner-configuration) in the Drivetrain section above. PathPlanner tuning depends entirely on having accurate drivetrain gains first.

---

## Quick Reference Table

| Mechanism | Motor(s) | Control | FF Terms | PID | Motion Profile | Constants File |
|-----------|----------|---------|----------|-----|---------------|----------------|
| Drive (steer) | TalonFX | Position | kS, kV | kP, kD | MotionMagicExpo | `TunerConstants` |
| Drive (drive) | TalonFX | Velocity | kS, kV | kP | — | `TunerConstants` |
| Turret | NEO | Position+MP | kS, kV, kA | kP | 1000°/s, 7200°/s² | `ShooterConstants.TurretConstants` |
| Hood | TalonFX | Position+MP | kS, kV | kP | 270°/s, 270°/s² | `ShooterConstants.HoodConstants` |
| Flywheel | 2× TalonFX | Velocity | kS, kV | kP | — | `ShooterConstants.FlywheelConstants` |
| Kicker | Vortex | Velocity (FF-only) | kS, kV | — | — | `ShooterConstants.KickerConstants` |
| Spindexer | NEO | Velocity | kS, kV | kP | — | `ShooterConstants.SpindexerConstants` |
| Intake Rollers | Kraken X60 | Velocity | kS, kV | kP | — | `IntakeConstants.Rollers` |
| Intake Pivot | 2× Vortex | Position+MP | kG, kS, kV | ⚠️ kP=0 | 90°/s, 90°/s² | `IntakeConstants.Pivot` |
| PathPlanner | — | Translation+Rot | — | kP | — | `Constants.PathPlannerConstants` + `Drive.java` |

### Priority Order for Monday

> **All FF, PID, and motion profile gains were set with limited testing time. Plan to re-tune everything Monday.**

0. **Physical measurements** — weigh robot with bumpers + battery, update mass in `Drive.java`, `settings.json`, and `Constants.DriveConstants`
1. **Drivetrain Steps 0-6** — everything else depends on accurate drive/odometry
   - Pre-flight checks (Step 0)
   - Steer PID verification (Step 1)
   - Drive FF characterization — run the "Simple FF" auto (Step 2)
   - Wheel radius characterization — run on carpet (Step 3)
   - Drive PID tuning (Step 4)
   - Max speed measurement (Step 5)
   - Slip current measurement (Step 6)
2. **Update PathPlanner values** (Step 8) — update mass in `Drive.java` to measured value, verify `settings.json` matches
3. **Turret** — needs to track accurately for shooting
4. **Flywheel** — RPM accuracy directly affects shot consistency
5. **Hood** — verify existing gains hold with the new LUT distances
6. **Intake Pivot** — uncomment FF, tune kG, then add PID
7. **Spindexer / Kicker / Rollers** — verify, adjust if feeding is inconsistent
8. **PathPlanner PID** — last, after drive is solid
