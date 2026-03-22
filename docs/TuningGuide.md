# PID / Feedforward / Motion Profile Tuning Guide

## Why Tuning Matters (read this first!)

**For mentors and build students who don't write code:** Tuning is the process of teaching the robot's software how your *specific* robot moves. Think of it like adjusting the brakes and steering on a car after you build it — the parts are all there, but they need to be calibrated to work together smoothly.

Every motor on the robot has "gains" — numbers that control how aggressively the software tries to reach a target (a position, a speed, etc.). If the gains are too low, the mechanism is sluggish and never quite gets where it needs to go. If they're too high, the mechanism overshoots, oscillates, or vibrates violently. **The right gains make the mechanism snap to its target quickly, hold steady, and not shake.**

**Why can't we just calculate the right numbers?** Because every robot is different. Friction in gearboxes, weight distribution, belt tension, wheel wear, carpet texture — all of these affect how the robot actually moves. Two identical robots built from the same BOM will need different gains. The only way to get it right is to put the robot on the ground and tune it.

**What does tuning look like?** Someone sits at a laptop with graphing software (AdvantageScope) while someone else watches the robot. You change a number, run the mechanism, look at the graph, and repeat. A well-tuned mechanism draws a clean, boxy shape on the graph — like a square wave. A poorly-tuned one draws wobbly curves or oscillates back and forth. It's surprisingly visual and intuitive once you see it.

**How long does it take?** Budget 2-4 hours for the drivetrain (Steps 0-6 below) and 30-60 minutes per additional mechanism. It goes faster with practice. This guide walks through everything step by step.

---

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
16. [Pigeon IMU Calibration](#pigeon-imu-calibration)
17. [PhotonVision Camera Tuning](#photonvision-camera-tuning)
18. [Elastic Dashboard for Matches](#elastic-dashboard-for-matches)
19. [Quick Reference Table](#quick-reference-table)

---

## General Principles

### Learning Resources

**If you're new to PID tuning,** work through these tutorials in order before touching the real robot. The sim practice is free and builds intuition fast.

| # | Resource | What You'll Learn | Link |
|---|----------|-------------------|------|
| 1 | **Mantik: PID Control** | What PID and feedforward are, in plain English | [mantik.netlify.app/frc/frc-pid-control](https://mantik.netlify.app/frc/frc-pid-control) |
| 2 | **Mantik: PID Practice — Setup** | How to set up YAMS sim + AdvantageScope for practice | [mantik.netlify.app/frc/pid-tuning-practice-setup](https://mantik.netlify.app/frc/pid-tuning-practice-setup) |
| 3 | **Mantik: PID Practice — Elevator** | Hands-on sim practice: find kG, then kP (doubling method) | [mantik.netlify.app/frc/pid-tuning-practice-elevator](https://mantik.netlify.app/frc/pid-tuning-practice-elevator) |
| 4 | **Mantik: PID Practice — Arm** | Hands-on sim practice: precise kG for arms, adding motion profiles | [mantik.netlify.app/frc/pid-tuning-practice-arm](https://mantik.netlify.app/frc/pid-tuning-practice-arm) |
| 5 | **Mantik: Trapezoidal Motion Profiling** | What motion profiles are and how to tune max velocity/acceleration | [mantik.netlify.app/frc/trapezoidal-motion-profiling](https://mantik.netlify.app/frc/trapezoidal-motion-profiling) |
| 6 | **Mantik: Exponential Motion Profiling** | Advanced profiles for arms (used by our intake pivot and steer motors) | [mantik.netlify.app/frc/exponential-motion-profiling](https://mantik.netlify.app/frc/exponential-motion-profiling) |
| — | **Mantik: AdvantageScope** | How to use our graphing/logging tool | [mantik.netlify.app/frc/advantagescope](https://mantik.netlify.app/frc/advantagescope) |
| — | **Mantik: Elastic Dashboard** | How to use our real-time dashboard | [mantik.netlify.app/frc/elastic-basics](https://mantik.netlify.app/frc/elastic-basics) |
| — | **YAMS Tuning Docs** | YAMS-specific tuning methodology | [yagsl.gitbook.io/yams](https://yagsl.gitbook.io/yams/) |
| — | **AK Swerve Template Docs** | Drivetrain-specific calibration steps | [docs.advantagekit.org](https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template/) |

> **Recommended path:** Read PID Control (#1) → do Elevator practice in sim (#3) → do Arm practice in sim (#4) → come back here and tune the real robot. Budget ~1 hour for the sim practice. It's worth it.

### Tuning Order (within any mechanism)

Every mechanism follows the same general pattern: **feedforward first, then PID.** Feedforward does the heavy lifting (80-90% of the work), and PID cleans up the remaining error.

> *Think of it like throwing a ball at a target. Feedforward is your aim — it gets the ball most of the way there. PID is the fine adjustment that nudges it the last few inches. If your aim (FF) is terrible, no amount of nudging (PID) will help.*

**Before you start — zero everything:**
- Set ALL gains to 0 (kP, kI, kD, kS, kV, kA, kG = 0).
- Set motion profile velocity and acceleration to **safe, low values**. You can increase later.
- Open AdvantageScope with a **Position vs Setpoint** graph (both lines on the same plot — this is your primary visual tool).

---

**For mechanisms that hold a position against gravity (turret, hood, intake pivot):**

| Step | Gain | What to do | What to look for |
|------|------|------------|-----------------|
| 1 | **kG** | Compensates for gravity so the mechanism holds still. **Binary search:** start low, double until it drifts up, then search between the last two values. For arms, get to **2-3 decimal places** (they're finicky). | The position line on the graph should be **perfectly flat** — not drifting up or down. |
| 2 | **kV** | Tells the motor how much voltage to apply per unit of speed. Start at 0.1 and increase in big jumps. | The **slope** (steepness) of the position line should **match the slope** of the reference line during the constant-speed part of the move. |
| 3 | **kA** | Handles acceleration (the curved start/end of moves). Start at 0.001, increase carefully. | The position line should **overlap the reference line** during the acceleration and deceleration phases. |
| 4 | **kP** | Corrects any remaining error. **Double from 0.1** (0.2, 0.4, 0.8, 1.6...) until you see slight overshoot, then back off 10-20%. | The graph should look like a **rectangle** — sharp rise, flat top, sharp fall. |
| 5 | **kD** | Dampens overshoot. Start tiny (0.005-0.05). Only needed for heavy or fast mechanisms. | Overshoot goes away. If you see jitter or hear buzzing, kD is too high — **back off**. |
| 6 | **kI** | **Almost never needed.** Corrects persistent small errors. Only add as a last resort. | Steady-state error disappears. ⚠️ Clear the integral accumulator before enabling kI. |

---

**For mechanisms that spin at a speed (flywheel, rollers, spindexer, kicker):**

| Step | Gain | What to do | What to look for |
|------|------|------------|-----------------|
| 1 | **kS** | Slowly increase voltage from 0 until the mechanism *barely* starts moving. That voltage is kS. | The mechanism just starts to move. *(Skip in simulation — sim motors have no friction.)* |
| 2 | **kV** | Set a target speed, adjust kV until actual speed matches. Test at several speeds. | The velocity line sits right on top of the setpoint line at steady state. |
| 3 | **kP** | Increase until the mechanism reaches setpoint within ~0.5 seconds without oscillation. | Fast spin-up, no wobble at steady state. |
| 4 | **kD** | Only if there's oscillation. Usually leave at 0 for velocity loops. | Oscillation stops. |

---

**How to tell if it's well-tuned (look at the position graph):**

| Graph Shape | Meaning |
|-------------|---------|
| ✅ **Rectangular/boxy** — sharp rise, flat top, sharp fall | Well-tuned! |
| ❌ **Slow, rounded curve** | kP too low, or feedforward isn't doing enough |
| ❌ **Overshoots then oscillates** | kP too high, or motion profile limits are too aggressive |
| ❌ **Slowly drifts up or down** | kG is wrong (gravity mechanisms) or kS is wrong (horizontal ones) |

> **Pro tip:** When tuning kV or kA, temporarily set kP to a high value to push the mechanism to the correct operating point, then set kP back to 0 before adjusting the FF gain. This ensures you're characterizing FF at the right position/speed.

### Motion Profiling: When and Which Type

A motion profile tells the motor controller "don't jump straight to the target — ramp up smoothly, cruise, then ramp down." Without it, the mechanism slams to full power instantly, which can break gears, skip belts, or ram into hard stops.

**Every mechanism that holds a position needs a motion profile.** Velocity mechanisms (flywheels, rollers) don't need one.

**We use two types:**

| Type | How it works | Used by |
|------|-------------|---------|
| **Trapezoidal** | Accelerates at a constant rate, cruises at max speed, decelerates at a constant rate. Simple and effective. | Turret, Hood |
| **Exponential** | Accelerates based on the motor's real voltage-speed curve (smoother, more realistic). Better for arms and precision mechanisms. | Drive steer, Intake pivot |

**To tune motion profile speed/acceleration** ([detailed guide](https://mantik.netlify.app/frc/trapezoidal-motion-profiling)):
1. Set acceleration very high. Increase max velocity until the mechanism struggles to keep up. Back off 10-20%.
2. Now increase acceleration until you see overshoot or vibration. Back off 15-25%.
3. If the graph shows overshoot at the corners, reduce acceleration. If motion feels slow, increase both.

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

YAMS lets you **change gains on the fly without redeploying code.** This is the fastest way to tune. Here's how:

1. **Switch Driver Station to "Test" mode** (not teleop or auto).
2. **Enable the robot.**
3. **Open Elastic Dashboard.** Find the Live Tuning button under `SmartDashboard > MECHANISM_NAME > Commands > Live Tuning`. Drag it onto your dashboard and click it.
4. **Edit gains** in the NetworkTables entries that appear — they take effect immediately.
5. **Watch the graph** in AdvantageScope to see the result.

**Available tuning entries** (published for every YAMS mechanism when `tuningMode = true`):

| What you can edit | Units | What it does |
|-------------------|-------|-------------|
| kP, kI, kD | — | PID gains (change takes effect instantly) |
| kS | volts | Static friction compensation |
| kV | volts per (unit/s) | Velocity feedforward |
| kA | volts per (unit/s²) | Acceleration feedforward |
| kG | volts | Gravity compensation (arms only) |
| Setpoint position | degrees | Where you want the mechanism to go |
| Setpoint velocity | RPM | How fast you want it to spin |
| Max velocity | RPM | Motion profile cruise speed |
| Max acceleration | RPM/s | Motion profile acceleration limit |

> ⚠️ **Safety first:** Always test in sim before the real robot. Accidentally typing kP = 300 when you meant 3.0 can cause violent oscillation.

> **Where to find the data:** Mechanism telemetry is under `NT:/Mechanisms`. Live Tuning commands are under `NT:/SmartDashboard`.

> **Units note:** YAMS works internally in rotations and rotations/sec, but the Live Tuning setpoint fields use **degrees** and **RPM** for convenience.

### AdvantageScope Tips

AdvantageScope is our graphing and log-replay tool. You'll use it constantly during tuning.

- **The key graph:** Drag both the `angle` (or `velocity`) and `setpoint` fields onto the same line graph. **The gap between the two lines tells you everything** — how fast the mechanism responds, whether it overshoots, and whether it holds steady.
- **Convert units:** Right-click an axis → "Edit Axis" → set multiplier to 360 (rotations → degrees) or 60 (RPS → RPM).
- **Mark your trials:** Press `N` to add a timestamp note like "kP=3.0" or "kP=5.0" so you can compare runs later.
- **Check for voltage saturation:** Plot voltage on a second Y-axis. If it's pegged at ±12V, the motor is maxed out and can't give more.
- **Export data:** Select a time range → right-click → Export to CSV. Useful for spreadsheet analysis.

### Elastic Dashboard Tips

Elastic is our real-time dashboard that shows live data from the robot.

- **"At setpoint" indicator:** We log `Mechanisms/HoodIsMoving`, etc. Add a boolean widget that turns green when the mechanism is at its target.
- **Group your tuning widgets:** Create a "Tuning" tab with gains on the left, a graph on the right, and current draw below.
- **Quick setpoint buttons:** Use tunable number widgets to jump between test positions (e.g. "0°", "15°", "30°" for the hood) without touching code.

### Phoenix Tuner X Tips (CTRE motors: drivetrain, hood, flywheel, intake rollers)

Phoenix Tuner X is CTRE's configuration and graphing tool. It connects directly to the motor controllers.

- **Position vs Reference overlay:** Plot `Position` and `ClosedLoopReference` on the same graph. The reference is where the motion profile thinks the mechanism should be *right now*. The gap tells you how well your gains are tracking.
- **Useful signal group:** Check `Position`, `ClosedLoopReference`, `ClosedLoopOutput`, and `StatorCurrent` → plot together for a complete picture.
- **Works in sim:** Connect to `localhost` when running sim to get the same plots on simulated motors.

### Position vs Reference: The Key Graph for kV/kA Tuning

When tuning kV and kA, you need to see two lines on the same graph:
- **Position** (or velocity) — where the mechanism *actually* is
- **Reference** — where the motion profile says it *should* be right now

This is NOT the final setpoint. The reference changes every cycle as the profile ramps up, cruises, and ramps down. Matching these two lines is how you tune kV and kA.

**For kV:** During the cruise phase (the straight part of the move), the two lines should have **the same slope**. If the position line is less steep, kV is too low.

**For kA:** During the acceleration/deceleration phases (the curved parts), the two lines should **overlap**. If the position lags behind during acceleration, kA is too low.

#### CTRE mechanisms (Hood, Flywheel) — ✅ Already set up

CTRE motors give us the reference signal for free via `getClosedLoopReference()`. **We already log this:**

| Mechanism | Position (AdvantageScope) | Reference (AdvantageScope) |
|---|---|---|
| **Hood** | `Hood/FX/PositionDegrees` | `Hood/FX/ReferenceDegrees` |
| **Flywheel** | `Flywheel/FX/VelocityRPM` | `Flywheel/FX/ReferenceRPM` |

Just open AdvantageScope, drag both onto the same graph, and you're ready to tune.

> **For future CTRE mechanisms**, follow the same pattern used in `HoodSubsystem.java` and `FlywheelSubsystem.java`: get the `positionSignal` and `referenceSignal` from the motor, refresh them together, and log both with `Logger.recordOutput()`.

#### REV mechanisms (Turret, Spindexer, Kicker) — ⚠️ No reference signal available

REV's SparkMax/SparkFlex **does not give us the reference signal.** When MAXMotion runs, the intermediate trajectory is computed inside the motor controller and never sent back to the RIO.

**Workaround — it's less precise but it works:**

1. **For kV:** Watch the actual position graph during the cruise phase. The mechanism should move at the motion profile's max velocity. If it moves slower, increase kV. Compare the slope of the position line against the expected speed: `expected_slope = max_velocity_degrees_per_second`.

2. **For kA:** Watch the shape of the acceleration phase. With kA=0, the curve will be "soft" (slow to accelerate, slow to decelerate). Increase kA until the transitions are crisp — sharp start, clean cruise, sharp stop.

> **Bottom line:** For our turret, this is fine — it's a simple horizontal mechanism where kV and kA can be dialed in by watching the position graph shape.

### Safety Checklist (before every tuning session)

- [ ] Robot on blocks (wheels off ground) for drivetrain tuning
- [ ] Current limits set in constants (already done for all subsystems)
- [ ] Soft limits enabled in YAMS configs (prevents over-travel on pivots)
- [ ] Someone ready on the E-stop
- [ ] `tuningMode = true` in `Constants.java` (currently `true`)

> **Telemetry note:** `tuningMode = true` automatically sets telemetry to HIGH (publishes all gains, setpoints, limits, temps). At competition, set `tuningMode = false` to switch to MID (less data, faster loop). See [YAMS Telemetry Verbosity Levels](#yams-telemetry-verbosity-levels).

---

## YAMS Telemetry Verbosity Levels

YAMS publishes different amounts of data depending on the verbosity level. Higher levels include everything from lower levels.

| Level | What it publishes | When to use |
|-------|------------------|-------------|
| **LOW** | Setpoints, sensor readings, mechanism position/velocity | Match play (minimal overhead) |
| **MID** | + Voltage, current draw, raw rotor data | Competition (enough to debug brownouts post-match) |
| **HIGH** | + All tunable gains (kP, kD, kV, etc.), limits, temps, motion profile params — all editable live | Shop tuning and practice |

**How it's configured:** `Constants.telemetryVerbosity()` returns `HIGH` when `tuningMode = true` and `MID` when `false`. Each motor at HIGH publishes ~30+ NetworkTables entries per cycle. With 10+ motors that's 300+ entries, so switching to MID at competition meaningfully reduces loop time.

| Situation | Level | `tuningMode` |
|-----------|-------|:---:|
| Shop tuning / practice | HIGH | `true` |
| Qualification/elimination matches | MID | `false` |
| Post-match replay | Whatever was logged | — |

---

## Top Team TunerConstants Comparison

How our swerve constants compare to top FRC teams in the **2026 season.** All use TalonFX (Kraken) swerve with Phoenix 6 unless noted.

> **Data sources:** Public GitHub repos reviewed June 2025: [6328](https://github.com/Mechanical-Advantage/RobotCode2026Public), [LASA PH2026](https://github.com/lasarobotics/PH2026), [LASA PR2026](https://github.com/lasarobotics/PR2026), [WHS 3467](https://github.com/WHS-FRC-3467/Skip-5.16-Platypus), [Hammerheads 5000](https://github.com/hammerheads5000/2026Rebuilt), [Lynk 9496](https://github.com/LynkRobotics/RobotCode2026Public).

### Hardware Summary

| Team | Module | Drive Ratio | Steer Ratio | Wheel Radius | Track/Wheelbase |
|------|--------|:-----------:|:-----------:|:------------:|:---------------:|
| **3216 (us)** | SDS MK4i | 4.667:1 | 26.09:1 | **1.80"** | — |
| 6328 Darwin | SDS MK5i R1 | 7.03:1 | 26.0:1 | 1.996" | — |
| 6328 Alphabot | SDS MK4i L3 | 6.12:1 | 21.43:1 | 1.880" | — |
| LASA (PH/PR) | — | 6.03:1 | 26.09:1 | 2.00" | — |
| WHS 3467 | — | 6.0:1 | 24.0:1 | 1.97" | 22" sq |
| Hammerheads 5000 | — | 6.03:1 | 26.09:1 | 1.985" | 22.5 × 20.75" |
| Lynk 9496 | SDS MK5n R2 | ~6.03:1 | 26.09:1 | ~2.0" | 20.75" sq |

**Takeaway:** 6.0–6.12 drive gear ratios are the most common. Our 4.667:1 is significantly faster-geared — this means higher theoretical top speed but less torque per amp. Our 1.80" wheel radius is slightly smaller than most teams; re-verify with the [wheel radius characterization](#step-3-wheel-radius-characterization).

### Current Limits

| Team | Drive Stator | Steer Stator | Slip Current |
|------|:-----------:|:-----------:|:------------:|
| **3216 (us)** | **80A** | **60A** | **120A** |
| 6328 | 80A | 40A | — (uses TorqueFOC) |
| LASA (PH/PR) | — | 60A | 120A |
| WHS 3467 | — | 60A | 94A |
| Hammerheads 5000 | 80A | 60A | 95A |
| Lynk 9496 | — | — | — |

**Takeaway:** 80A drive / 60A steer is the consensus. Our **120A slip current matches LASA but is higher than most.** WHS and Hammerheads both measured ~95A. Run the slip test ([Step 6](#step-6-slip-current-measurement)) to find our actual value.

### Drive/Steer Gains — Voltage Mode

Only comparing teams using Voltage mode (apples-to-apples). Teams using TorqueCurrentFOC have wildly different gain scales — see separate table below.

| Team | Steer kP | Steer kD | Steer kS | Steer kV | Drive kP | Drive kS | Drive kV |
|------|:-------:|:-------:|:-------:|:-------:|:-------:|:-------:|:-------:|
| **3216 (us)** | **100** | **0.5** | **0.1** | **2.48** | **0.1** | **0.26** | **0.585** |
| LASA (PH/PR) | 100 | 0.5 | 0.1 | 2.49 | 0.1 | 0 | 0.124 |
| Hammerheads (generated) | 100 | 0.5 | 0.1 | 2.49 | 0.1 | 0 | 0.124 |
| Hammerheads (tuned) | 1000 | 8 | 6 | 0 | 2 | 0.237 | 0.733 |

> **Note:** Hammerheads has both a generated `TunerConstants.java` (matching LASA exactly — likely default Tuner X output) and hand-tuned `SwerveConstants` in their `Constants.java` with much higher steer gains. Their tuned steer uses TorqueCurrentFOC while drive stays Voltage.

**Takeaway:** Steer kP=100 / kD=0.5 / kS=0.1 / kV≈2.49 in Voltage mode is the standard starting point across multiple teams. Our steer gains match the consensus. Our drive kV (0.585) is higher than LASA (0.124) — this is expected because of our much lower gear ratio (4.667 vs 6.03); kV scales inversely with gear ratio. **After wheel radius characterization, re-run drive FF characterization to verify.**

### Drive/Steer Gains — TorqueCurrentFOC Mode

These gains are in **amps**, not volts. Do not mix with Voltage mode gains.

| Team | Steer kP | Steer kD | Drive kP | Drive kS |
|------|:-------:|:-------:|:-------:|:-------:|
| 6328 Darwin | 4000 | 50 | 35 | 5.0 |
| WHS 3467 | 2000 | 20 | 60 | 7.26 |
| Hammerheads (steer only) | 1000 | 8 | — | — |

**Takeaway:** TorqueFOC gains are 10–40× larger than Voltage gains. If we ever switch to TorqueFOC, 6328 and WHS 3467 provide good reference starting points. See [TorqueFOC section](#torquefoc--available-upgrade-path).

### Control Mode

| Team | Steer Output | Drive Output | Phoenix Pro? |
|------|:-----------:|:-----------:|:---:|
| **3216 (us)** | **Voltage** | **Voltage** | **Yes** |
| 6328 | TorqueCurrentFOC | TorqueCurrentFOC | Yes |
| LASA (PH/PR) | Voltage | Voltage | — |
| WHS 3467 | TorqueCurrentFOC | TorqueCurrentFOC | Yes |
| Hammerheads | TorqueCurrentFOC | Voltage | Yes |
| Lynk 9496 | (custom) | (custom) | — |

**Voltage mode is still the most common approach.** 6328 and WHS 3467 fully committed to TorqueFOC. Hammerheads uses a split approach (TorqueFOC steer + Voltage drive). We can switch later — see [TorqueFOC section](#torquefoc--available-upgrade-path).

### Max Speed

| Team | kSpeedAt12Volts | Drive Ratio | Notes |
|------|:--------------:|:-----------:|-------|
| **3216 (us)** | **6.02 m/s** | 4.667:1 | Theoretical — needs measurement |
| 6328 Darwin | 4.20 m/s | 7.03:1 | Measured |
| 6328 Alphabot | 4.69 m/s | 6.12:1 | Measured |
| LASA (PH/PR) | 5.12 m/s | 6.03:1 | |
| WHS 3467 | 4.37 m/s | 6.0:1 | |
| Hammerheads | 5.85 m/s | 6.03:1 | |
| Lynk 9496 | 5.12 m/s | ~6.03:1 | |

**Takeaway:** Measured speeds are typically 10-30% below theoretical. Our 6.02 m/s theoretical will likely measure around 5.0-5.5 m/s. Run the [max speed test](#step-5-max-speed-measurement) and update.

### Pigeon 2 Configuration

| Team | MountPose Configured? | Roll | Pitch | Yaw |
|------|:--------------------:|:----:|:-----:|:---:|
| **3216 (us)** | **No** | — | — | — |
| LASA PR2026 | ✅ | 179.96° | 1.14° | -90.45° |
| WHS 3467 | Partial | 180° | — | — |
| Hammerheads | ✅ | — | — | -90° |
| Others | `null` (skipped) | — | — | — |

**Takeaway:** If our Pigeon is mounted in any orientation other than flat + forward-facing, we need to configure `MountPose`. See the new [Pigeon IMU Calibration](#pigeon-imu-calibration) section below.

### Interesting Patterns From Top Teams

| Pattern | Who Uses It | What It Does |
|---------|-------------|-------------|
| **LoggedTunableNumber** | 6328, WHS 3467, Hammerheads | Runtime-tunable gains with `ifChanged()` callback — only applies to motors when the value actually changes. More efficient than YAMS live tuning but requires more boilerplate. |
| **WheelSlipAuto** | WHS 3467 | Dedicated auto routine that ramps motor voltage until target velocity is reached, logging current vs velocity for slip characterization. More automated than our wall test. |
| **Separate sim constants** | WHS 3467, Hammerheads | Different kP/kV values for sim vs real modules. Our sim uses CTRE's built-in sim, but adding sim-specific PID could improve simulation accuracy. |
| **SwerveSetpointGenerator** | BroncBotz 3481 | PathPlanner's port of 254's setpoint generator — limits acceleration per-module for smoother motion. We mention this in our guide but haven't enabled it. |
| **Wheel COF parameter** | 6328 (1.5), Hammerheads (2.255) | Coefficient of friction used for PathPlanner's traction model. Our 1.2 is conservative — consider measuring. |
| **Open loop ramps** | LASA PR (0.5s), Lynk (0.25s) | Voltage ramp rates to reduce wheel slip on acceleration. We don't use ramps currently. |
| **driveWithSetpointGenerator** | BroncBotz 3481 | 254's setpoint generator integrated with PathPlanner for smoother autonomous driving. |

### Shooter Mechanism Gains (Reference)

Several top teams in 2026 also run flywheel + hood shooters. Use this as a sanity check when tuning our gains.

| Team | Flywheel kP | Flywheel kS | Flywheel kV | Hood kP | Hood kD | Hood kS |
|------|:----------:|:----------:|:----------:|:------:|:------:|:------:|
| **3216 (us)** | **0.2** | **0.35** | **0.12** | **300** | **0** | **0.45** |
| 6328 | 0.4 | 0.22 | 0.019 | 1200 | 4 | — |
| WHS 3467 | 12.0 | 4.0 | — | 3000 | 160 | 8.0 |
| Hammerheads 5000 | 12.0 | 6.0 | 0.04 | 800 | 5 | 0.28 |

> **⚠️ Control mode matters!** WHS 3467 and Hammerheads use **TorqueCurrentFOC** (gains in amps), while 6328 uses TorqueFOC too. Our gains are in **Voltage** mode — do not directly copy their numbers. The relative ratios (kP vs kS) are more instructive than the absolute values.

| Team | Kicker kP | Kicker kS | Kicker kV | Turret kP | Notes |
|------|:--------:|:--------:|:--------:|:--------:|-------|
| **3216 (us)** | **0** | **0.25** | **0.12** | **3.0** | FF-only kicker (SparkFlex) |
| 6328 | 3.0 | 0.5 | 0.09 | — | Kicker PID + FF |
| BroncBotz 3481 | — | 0.18 | 0.62 | — | FF-only kicker |
| Hammerheads 5000 | — | — | — | 200 | TorqueFOC turret |

**Takeaway:** Our flywheel kP (0.2) is at the low end. If the flywheel is slow to recover after a shot, try increasing to 0.5-1.0. Our hood kP (300) is reasonable for Voltage mode; 6328's 1200 uses TorqueFOC. Kicker FF-only (kP=0) is common — only add PID if the kicker stalls on ball contact.

> **Action items from research:** (1) Measure slip current, (2) Configure Pigeon MountPose if needed, (3) Consider adding open loop ramps (0.25s) for smoother acceleration, (4) Measure wheel COF for PathPlanner.

---

## TorqueFOC — Available Upgrade Path

> **You don't need this right now.** Start with Voltage mode (what we're using), get the robot driving well, then consider TorqueFOC as a future upgrade.

TorqueFOC (TorqueCurrentFOC) is a Phoenix Pro feature where the motor controller commands **current** (amps) instead of voltage. This gives more consistent performance as the battery drains, and more predictable traction control.

**We have Phoenix Pro and the code is already wired for it.** Switching is a one-line config change, but it requires a **complete retune** of every drive gain.

### Why consider it (later)

- Performance stays consistent even as the battery sags from 12V → 11V
- Better traction control (slip current directly limits torque)
- 6328 (Mechanical Advantage) uses it; most other top teams use Voltage

### What changes

| | Voltage Mode (current) | TorqueFOC |
|---|---|---|
| Steer kP | 100 | ~4000 |
| Steer kD | 0.5 | ~50 |
| Drive kP | 0.1 | ~35 |
| kS/kV units | Volts | Amps |

**All gains change dramatically.** You cannot reuse Voltage-mode gains.

### How to switch (when ready)

1. In `TunerConstants.java`, change both output types to `ClosedLoopOutputType.TorqueCurrentFOC`
2. Zero out all gains and start fresh
3. Re-run the entire drivetrain calibration (Steps 1–7)
4. Optionally add a torque ramp (0.02s) to prevent current spikes

### Motion Profile Types by Mechanism

Each mechanism uses a different profile type. Here's what they are and why:

| Mechanism | Profile Type | Why |
|-----------|-------------|-----|
| Drive (steer) | **MotionMagicExpo** (firmware) | Smooth, continuous motion — good for steering |
| Turret | **MAXMotion trapezoidal** (firmware) | Simple point-to-point moves |
| Hood | **MotionMagic trapezoidal** (firmware) | Small mechanism, short moves |
| Intake Pivot | **ExponentialProfile** (RIO) | Accounts for gravity — arm accelerates faster going down than up |

> **Simple rule:** Trapezoidal = constant acceleration. Exponential = acceleration changes with speed (more realistic). Use exponential for steering and gravity-loaded arms, trapezoidal for everything else.

---

## YAMS FF + Motion Profile Requirement

> **Key rule:** For position-controlled mechanisms (turret, hood, intake pivot), **kV and kA only work if a motion profile is configured.**

### Why

kV multiplies the *velocity the profile says the mechanism should be moving at right now*. Without a profile, there's no planned velocity — so kV × 0 = 0. The feedforward does nothing.

**All our positional mechanisms already have profiles, so this is already correct:**

| Mechanism | Has FF? | Has Motion Profile? | ✅ |
|-----------|:-------:|:-------------------:|:--:|
| Turret | kS=0, kV=1.0, kA=0.05 | 1000°/s, 7200°/s² | ✅ |
| Hood | kS=0.45, kV=3.0 | 270°/s, 270°/s² | ✅ |
| Intake Pivot | kG=0.21, kS=0.11 | 90°/s, 90°/s² | ✅ |

**If you add FF to a new positional mechanism in the future, always pair it with a motion profile.**

### Velocity mechanisms are unaffected

Flywheel, kicker, spindexer, and intake rollers use velocity control — the setpoint itself is a velocity. kV multiplies the velocity setpoint directly, no profile needed.

---

## YAMS SysId Helpers

YAMS mechanisms have a built-in `sysId()` method that creates a complete characterization routine with one call. This gives you starting values for kS, kV, kA, and kG.

| Approach | Best for | Use on |
|----------|----------|--------|
| **SysId** | Getting kS, kV, kA, kG to within ~5% | Intake pivot (kG), flywheel (kV), turret (kV) |
| **Manual** | Quick iteration when you have a ballpark | Hood, kicker, spindexer |
| **YAMS Live Tuning** | Fine-tuning after SysId gives starting values | All mechanisms |

**REV note:** YAMS uses duty cycle internally for cleaner SysId data. REVLib 2026+ auto-writes `.revlog` files you can open in AdvantageScope.

**CTRE note:** YAMS uses `VoltageOut` for SysId. You need to start/stop Signal Logger manually (bind to controller buttons). Extract `.hoot` logs via Phoenix Tuner X → Tools → Extract Signal Logs.

---

## 1. Drivetrain (CTRE Swerve)

**File:** `TunerConstants.java` (generated by Phoenix Tuner X — use Tuner X to regenerate, don't hand-edit gains)
**Motors:** 8× TalonFX (4 drive + 4 steer), **CAN FD bus** at 250 Hz odometry
**Control:** Phoenix 6 Voltage mode (TorqueFOC available later — see [TorqueFOC section](#torquefoc--available-upgrade-path))

> ⚠️ **AdvantageKit uses different gain scales than CTRE's default swerve code.** AK applies the gear ratio in firmware, so FF and PID gains from other projects won't work directly. Use the AK characterization routines below.

> **AdvantageScope layout:** Import `AdvantageScope Swerve Calibration.json` from the swerve project folder for predefined tuning tabs.

### Enabling Characterization Routines

1. Uncomment `setupSysid();` in the `RobotContainer` constructor (line 224)
2. Deploy and reboot
3. The dashboard auto chooser will show characterization options
4. **Re-comment `setupSysid()` before competition**

### Complete Calibration Order

Follow this order — each step depends on the previous one:

| Step | What | Uses Auto Routine? |
|:----:|------|:------------------:|
| 0 | [Pre-flight checks](#step-0-pre-flight-checks) | — |
| 1 | [Steer PID](#step-1-steer-pid) | — |
| 2 | [Drive FF (kS, kV)](#step-2-drive-feedforward-characterization-ks-kv) | ✅ |
| 3 | [Wheel radius](#step-3-wheel-radius-characterization) | ✅ |
| 4 | [Drive PID (kP)](#step-4-drive-pid-tuning-kp) | — |
| 5 | [Max speed](#step-5-max-speed-measurement) | — |
| 6 | [Slip current](#step-6-slip-current-measurement) | — |
| 7 | [Odometry frequency](#step-7-odometry-frequency) | — |
| 8 | [PathPlanner config](#step-8-pathplanner-configuration) | — |

### Pre-Tuning Physical Measurements

Take these measurements before any software calibration:

| Measurement | How | Current Value | Status |
|---|---|---|---|
| **Robot mass** (with bumpers + battery) | Scale | **63.5 kg (140 lbs)** | ✅ Updated (verify on scale) |
| **Wheel radius** (effective) | Software — [Step 3](#step-3-wheel-radius-characterization) | 1.8" (nominal) | ⚠️ Needs characterization on carpet |
| **Max speed** | Software — [Step 5](#step-5-max-speed-measurement) | 6.02 m/s (theoretical) | ⚠️ Needs measurement |
| **Slip current** | Software — [Step 6](#step-6-slip-current-measurement) | 120A (default) | ⚠️ Needs wall test |
| **Robot MOI** | PathPlanner GUI estimator | 6.883 kg⋅m² | ⚠️ Verify with auto paths |

> **Keep three locations in sync** after every measurement: `Drive.java`, `settings.json` (PathPlanner), and `Constants.DriveConstants`.

### Step 0: Pre-flight Checks

Before any tuning:

- [ ] **Phoenix Tuner X connected** — all 8 TalonFX + 4 CANcoders show green
- [ ] **Module offsets correct** — all wheels point straight forward with offset ≈ 0°
- [ ] **Motor inversions correct** — forward spin = positive velocity, CCW steer = positive position
- [ ] **CAN bus healthy** — 0% error rate, all on `CANFD` bus
- [ ] **Firmware updated** — latest compatible with Phoenix 6 v26
- [ ] **Robot on blocks** for Steps 0–2 and 4. Wheels on ground for Steps 3, 5, 6.

### Step 1: Steer PID

Current gains: `kP=100, kI=0, kD=0.5, kS=0.1, kV=2.48, kA=0`

The steer motors use MotionMagicExpo (exponential profile) — this runs in firmware automatically.

**How to tune (robot on blocks):**

1. **kS:** Command a very slow steer rotation. Increase kS from 0 until the module barely starts moving. That voltage is kS. (Currently 0.1)
2. **kV:** Command a moderate speed. `kV = (voltage - kS) / velocity`. Overlay position vs reference — slopes should match during cruise. (Currently 2.48)
3. **kP:** Command 0° → 90°. Start at ~50, increase until quick settling with no oscillation. Target: **<50ms**. (Currently 100 — standard across all top teams)
4. **kD:** If there's overshoot, add kD in small steps (0.1–1.0). (Currently 0.5)
5. **kI:** Leave at **0**. Steer doesn't need integral.

**Verify in AdvantageScope:** Plot `/RealOutputs/SwerveStates/Measured` and `/RealOutputs/SwerveStates/SetpointsOptimized` — measured angles should track setpoints with no oscillation.

### Step 2: Drive Feedforward Characterization (kS, kV)

Current gains: `kP=0.1, kI=0, kD=0, kS=0.25955, kV=0.58499`

> **Use the AK characterization routine** — don't copy FF values from non-AK projects (different gain scales).

**Quick method — "Drive Simple FF Characterization" auto (recommended):**

1. Open space, wheels on ground, fully charged battery
2. Select **"Drive Simple FF Characterization"** from the auto chooser
3. Enable auto — robot drives forward with a slow ramp
4. Let it run 5–10 seconds, then disable
5. Check the Driver Station console for the measured kS and kV
6. Update `driveGains` in `TunerConstants.java`
7. Repeat 2–3 times and average for accuracy

> **Expected range:** kV ≈ 0.4–0.7, kS ≈ 0.1–0.4 for Kraken X60 with 4.667:1 and AK firmware-applied ratio.

**Alternative — Full SysId** (also gets kA):

Run the four SysId routines (quasistatic/dynamic, forward/reverse), then analyze with the WPILib SysId tool. AK logs are in WPILOG format in the `logs/` directory (positions in radians, not rotations).

### Step 3: Wheel Radius Characterization

Current value: `kWheelRadius = Inches.of(1.8)` (nominal — needs characterization)

**Procedure:**

1. Place the robot **on carpet** (not hard floor — carpet compresses the wheel differently)
2. Select **"Drive Wheel Radius Characterization"** from the auto chooser
3. Enable auto — robot slowly rotates in place
4. Let it run 2–3 full rotations, then disable
5. Check the Driver Station console for the measured wheel radius
6. Update `kWheelRadius` in `TunerConstants.java`

> **Expected:** ~1.70–1.85" for our 3.6" diameter wheels. Re-run when you swap wheels or move to different carpet.

### Step 4: Drive PID Tuning (kP)

After FF is set and wheel radius is accurate:

1. Deploy with new FF values
2. In teleop, do a sudden joystick push (0 → ~2 m/s)
3. In AdvantageScope, plot measured vs setpoint velocities
4. Increase kP until rise time is <100ms with no oscillation. Start at 0.05, try 0.1, 0.2, etc.
5. **kD: Usually 0** for velocity loops — only add if you see ringing
6. Current kP=0.1 is a good starting point

> **If oscillating:** Reduce kP. Steady-state velocity error = FF problem (kV wrong), not PID.

### Step 5: Max Speed Measurement

Current value: `kSpeedAt12Volts = 6.02 m/s` (theoretical — needs real measurement)

This value normalizes joystick input. If it's too high, the robot can never reach full speed. If too low, the joystick saturates early.

**Procedure:**

1. Open space (30+ feet), fully charged battery
2. Drive **full speed forward**, hold for 3+ seconds until velocity stabilizes
3. In AdvantageScope, read the peak sustained velocity (not the spike, the plateau)
4. Update `kSpeedAt12Volts` in `TunerConstants.java` and `"maxDriveSpeed"` in `settings.json`

> **Expected:** ~5.5–5.9 m/s (vs 6.02 theoretical). The gap is friction, motor efficiency, and battery sag.

### Step 6: Slip Current Measurement

Current value: `kSlipCurrent = 120A` (most top teams use 80A — needs verification)

Slip current is the stator current where the wheels lose traction and start spinning. It's used for traction control.

**Procedure:**

1. Push the robot against a **solid, immovable wall**, all wheels on carpet
2. In AdvantageScope, plot drive current and drive velocity for one module
3. **Slowly** push the joystick from 0% → 100% forward (into the wall)
4. Watch velocity — it'll be ~0 (gripping), then **suddenly jump** (slipping)
5. The current at that moment = your slip current
6. Repeat 2–3 times, average, and update `kSlipCurrent` in `TunerConstants.java`

> **Expected:** 60–100A for Kraken X60 on competition carpet. Most top teams use 80A. Our 120A is likely too high.

> **Note:** If `kSlipCurrent` (120A) is above the stator current limit (80A), the traction control never activates because the motor can't reach slip current. After measuring, make sure `kSlipCurrent` reflects the actual slip point.

### Step 7: Odometry Frequency

**No action needed.** Our CAN FD bus automatically runs odometry at 250 Hz (set in `Drive.java`). This is the AdvantageKit recommendation.

Only change this if you see loop overruns (reduce to 200 Hz), but that's rare.

### Step 8: PathPlanner Configuration

PathPlanner needs accurate physical parameters. These live in **two places that must match:**

| Parameter | `Drive.java` (runtime) | `settings.json` (PathPlanner GUI) | Status |
|---|---|---|---|
| Robot mass | 63.503 kg | 63.503 | ✅ |
| Robot MOI | 6.883 kg⋅m² | 6.883 | ⚠️ Verify with auto paths |
| Wheel COF | 1.2 | 1.2 | ✅ |
| Wheel radius | 0.04572 m | 0.04572 | ⚠️ Re-characterize (Step 3) |
| Max speed | 6.02 m/s | 6.02 | ⚠️ Re-measure (Step 5) |
| Drive current limit | 80A | 80A | ✅ |

**PathPlanner PID** (`Constants.PathPlannerConstants`): Translation kP=5.0, Rotation kP=5.0

1. **Tune drivetrain first** — PathPlanner PID sits on top of drive gains
2. **Translation kP:** Run a straight path. If the robot drifts, increase. If it oscillates, decrease.
3. **Rotation kP:** Run a path with heading changes. Slow to rotate = increase. Oscillates = decrease.

**MOI estimation:** If auto paths overshoot rotations → MOI too low. Sluggish to rotate → MOI too high. Current 6.883 is reasonable for a robot with a heavy center turret.

### Advanced: Profiled Turning & Swerve Setpoint Generator

> **You don't need these right now.** The steer motors already use MotionMagicExpo for smooth azimuth transitions. PathPlanner also has 254's swerve setpoint generator algorithm available, but it's not enabled — consider it only after all basic calibration is complete.

---

## 2. Turret

**Motor:** SparkMax + NEO (27:1) · **Control:** YAMS Pivot (position + MAXMotion) · **FF:** SimpleMotorFeedforward(kS, kV, kA)

Current gains: `kP=3.0, kD=0, kS=0, kV=1.0, kA=0.05`
Motion profile: `1000 °/s, 7200 °/s²`

> ⚠️ Motion profile was reduced from 1440°/s — NEO through 27:1 maxes out at ~1261°/s.

**How to tune** (follows the [standard tuning order](#tuning-order-within-any-mechanism)):

1. **kS:** Horizontal turret, so no gravity term needed. Find the minimum voltage to start the turret moving. Currently 0.
2. **kV:** Command a move, watch Position vs Reference in AdvantageScope. Match the slopes during cruise. Currently 1.0.
3. **kA:** Match the curved acceleration portions. Start at 0.001. Currently 0.05.
4. **kP:** Command 0° → 90°, increase until settling < 200ms. Currently 3.0.
5. **kD:** Add if overshoot. Start at 0.05.
6. **Motion profile:** 1000°/s = ~0.36s for a full 360°. Increase toward 1200°/s if too slow (stay under 1261°/s).

**AdvantageScope keys:** `Shooter/Turret/angle`, `Shooter/Turret/setpoint`

---

## 3. Hood

**Motor:** TalonFX (Kraken X44, 30:1) · **Control:** YAMS Pivot (position + MotionMagic) · **FF:** SimpleMotorFeedforward(kS, kV, kA)

Current gains: `kP=300, kD=0, kS=0.45, kV=3.0, kA=0`
Motion profile: `270 °/s, 270 °/s²`

**How to tune:**

1. **kS:** Already 0.45. Verify with a slow sweep — should move smoothly from standstill.
2. **kV:** Overlay Position vs Reference in Phoenix Tuner X. Match slopes during cruise. kV is actively used by MotionMagic.
3. **kP:** High kP (300) because it's small and needs precise LUT angles. Command 0° → 15° → 0°, target settling < 100ms. Increase until slight overshoot, then back off.
4. **kD:** Only add if you hear audible oscillation/chatter. Be conservative — low-inertia mechanisms amplify noise.
5. **Motion profile:** 270°/s is moderate. Increase if sluggish during rapid LUT changes, decrease if overshooting.

**AdvantageScope keys:** `Hood/angle`, `Hood/setpoint`

---

## 4. Flywheel

**Motors:** 2× TalonFX (leader + inverted follower) · **Control:** YAMS FlyWheel (velocity) · **FF:** SimpleMotorFeedforward(kS, kV, kA)

Current gains: `kP=0.2, kD=0, kS=0.35, kV=0.12, kA=0`

**How to tune:**

1. **kS:** Slowly ramp voltage until the flywheel barely starts spinning. Record that voltage. *(Skip in sim — sim has no static friction.)*
2. **kV:** Set a target velocity (kP=0), adjust kV until steady-state velocity matches. Test at several speeds (1000, 2000, 3000, 4000 RPM).
3. **kP:** Command a step velocity, increase until the flywheel reaches setpoint within ~0.5s without oscillation. Currently 0.2.
4. **kA (optional):** Only needed if you want faster spin-up.
5. **Tolerance:** `kVelocityTolerance = 30 RPM`. Tighten if shots are inconsistent (costs spin-up time).

**AdvantageScope keys:** `Flywheel/velocity`, `Flywheel/setpoint`
**Dashboard tunable:** `Shooter/FlywheelRPM`

---

## 5. Kicker

**Motor:** SparkFlex + NEO Vortex · **Control:** YAMS FlyWheel (FF-only, PID = 0) · **FF:** SimpleMotorFeedforward(kS, kV, kA)

Current gains: `kP=0.0, kS=0.25, kV=0.12`

The kicker just needs to spin at a consistent speed to feed balls — it doesn't need tight tracking. Tune kS/kV like the flywheel. Only add kP if the kicker slows noticeably when a ball enters.

**Dashboard tunable:** `Kicker/KickerRPM`

---

## 6. Spindexer

**Motor:** SparkMax + NEO (5:1) · **Control:** YAMS FlyWheel (velocity) · **FF:** SimpleMotorFeedforward(kS, kV, kA)

Current gains: `kP=0.02, kS=0.25, kV=0.6`

Tune kS/kV like flywheel/kicker. kP=0.02 is very low — increase if balls feed inconsistently (spindexer slows too much under load). Uses COAST idle mode (freewheels when not commanded — intentional).

**Dashboard tunable:** `Spindexer/IndexerRPM`

---

## 7. Intake Rollers

**Motor:** TalonFX (Kraken X60, 2:1) · **Control:** YAMS FlyWheel (velocity) · **FF:** SimpleMotorFeedforward(kS, kV, kA)

Current gains: `kP=0.5, kS=0.39, kV=0.24`

Tune kS/kV like other velocity mechanisms. kP=0.5 is relatively aggressive — verify by running the intake and feeding balls. If rollers slow down significantly on ball contact, increase kP. If they buzz/oscillate, decrease.

---

## 8. Intake Pivot

**Motors:** 2× SparkFlex + NEO Vortex (30:1) · **Control:** YAMS Arm (position + ExponentialProfile) · **FF:** ArmFeedforward(kS, kG, kV) — gravity-compensated

Current gains: `kP=0.0, kG=0.21, kS=0.11, kV=0, kA=0`
Motion profile: `90 °/s, 90 °/s²`

> ⚠️ **PID is all zeros and feedforward is commented out.** First priority on Monday: uncomment `.withFeedforward(armFeedforward())`, tune kG, then add PID.

**How to tune:**

1. **kG (gravity compensation — do this first!):** Hold the arm horizontal (0° from horizontal). Increase kG until it holds position. YAMS applies `kG × cos(angle)` automatically. Currently 0.21.
2. **kS/kV:** Run a slow constant-velocity sweep. `kV = (voltage - kG×cos(θ) - kS) / velocity`. Currently kV=0 — should be nonzero after tuning.
3. **kP:** Command stow → deploy. Increase until settling < 0.3s. Start at 1.0.
4. **kD:** The arm has significant inertia — expect to need some kD (start at 0.05).
5. **Motion profile:** 90°/s is conservative. Increase after PID is stable for faster deploy/stow.

**AdvantageScope keys:** `IntakePivot/angle`, `IntakePivot/setpoint`

---

## 9. PathPlanner

See [Step 8: PathPlanner Configuration](#step-8-pathplanner-configuration) in the Drivetrain section above. PathPlanner tuning depends entirely on having accurate drivetrain gains first.

---

## Pigeon IMU Calibration

The Pigeon 2 IMU is the robot's gyroscope — it tells the code which direction the robot is facing. If it's not calibrated correctly, **everything that depends on field-relative driving, autonomous paths, and vision pose estimation will be wrong.**

### Why MountPose Matters

The Pigeon 2 has a default orientation: it expects to be mounted flat on the robot with its arrow pointing toward the front. If it's mounted in any other orientation (rotated, upside-down, sideways), you need to tell it how it's mounted using `MountPose`.

From the 2026 research:
- **LASA PR2026:** `MountPose(roll=179.96°, pitch=1.14°, yaw=-90.45°)` — nearly upside-down, rotated 90°
- **WHS 3467:** `MountPose(roll=180°)` — upside-down, arrow forward
- **Hammerheads 5000:** `MountPose(yaw=-90°)` — flat, arrow pointing left

### How to Configure MountPose

1. **Physically inspect the Pigeon 2** on the robot:
   - Which way does the arrow point? (Front = 0° yaw, Left = -90° yaw, etc.)
   - Is it mounted flat? (Flat = 0° roll/pitch; upside-down = 180° roll)
2. **Open Phoenix Tuner X** → Select the Pigeon 2 device
3. **Go to the "Configs" tab** → Under `MountPose`, enter:
   - **MountPoseYaw:** Degrees from forward. Arrow pointing left = -90°. Arrow pointing right = 90°.
   - **MountPoseRoll:** 0° if flat/right-side-up. 180° if upside-down.
   - **MountPosePitch:** Usually 0° unless the Pigeon is tilted forward/back.
4. **Apply and save to device**

These values are also configurable in code via `Pigeon2Configuration`:
```java
var pigeonConfig = new Pigeon2Configuration();
pigeonConfig.MountPose.MountPoseYaw = -90;  // arrow points left
pigeonConfig.MountPose.MountPoseRoll = 180; // upside-down
pigeonConfig.MountPose.MountPosePitch = 0;
```

### Temperature Calibration

The Pigeon 2 drifts slightly as temperature changes. It has an automatic temperature compensation feature, but it works best when:

1. **Power on the robot and let it sit for 2-3 minutes** before driving — the Pigeon auto-calibrates on startup.
2. **Avoid bumping or moving the robot during startup** — the gyro calibrates during the first few seconds.
3. **Zero the yaw** before the match starts. AdvantageKit's swerve template calls `pigeon.setYaw(0)` during gyro reset — this happens when the driver presses the reset button.

### Verification

After configuring MountPose:
1. Place the robot facing a known direction (e.g., away from your alliance wall)
2. Open AdvantageScope → look at the `/Drive/Gyro/YawPosition` value
3. Turn the robot 90° left — it should read approximately +90°
4. Turn it 90° right from starting position — it should read approximately -90°
5. If the signs are flipped or the values are wrong, your MountPose is incorrect

> **⚠️ If the Pigeon is mounted flat with the arrow pointing forward, you don't need to set MountPose.** Check your physical mounting before adding configuration.

---

## PhotonVision Camera Tuning

PhotonVision provides pose estimation using AprilTags. Camera settings affect how well it detects tags, especially under varying lighting (bright arena vs. dark practice space).

### Resolution vs. FPS Tradeoff

Higher resolution detects tags at longer range but processes fewer frames per second:

| Resolution | Detection Range | Typical FPS | Best For |
|:----------:|:--------------:|:-----------:|----------|
| 640×480 | Short (~3m) | 60+ fps | Fast tracking, close range |
| 1280×960 | Medium (~5m) | 20-30 fps | Good balance (recommended) |
| 1600×1200 | Long (~7m) | 10-15 fps | Maximum range but slow |
| 1800×1200 | Long (~7m+) | 8-12 fps | 6328 uses this for mono cameras |

**Recommendation:** Start with **1280×960** for a good balance. Only increase if you need longer detection range and can tolerate slower updates.

### 6328's Camera Configuration (Reference)

From their 2026 code:
- **Color cameras:** 1280×960, 90° FOV, exposure=4500µs, gain=5 → fast detection, good color
- **Mono cameras:** 1800×1200, 75° FOV, exposure=1800µs, gain=15 → max range, high gain to compensate for monochrome

### Key Settings (PhotonVision Web Interface)

Access at `http://photonvision.local:5800` when connected to the robot:

1. **Exposure:** Lower = sharper images (less motion blur), but darker. Start around **3000-5000µs** for arena lighting. Reduce if tags are blurry during fast movement.
2. **Gain:** Amplifies the sensor signal. Higher gain = brighter image but more noise. Start at **5** for color cameras, **10-15** for monochrome.
3. **Brightness/Contrast:** Leave at defaults unless the arena is unusually bright or dark.
4. **LED Mode:** Keep LEDs **OFF** for AprilTag detection — they can cause glare and wash out the tags.
5. **Target model:** Make sure it's set to **AprilTag 36h11** (the FRC standard).

### Multi-Camera Considerations

If running multiple cameras:
- Each camera adds CPU load to the coprocessor. Monitor CPU usage in the PhotonVision web interface.
- Stagger camera resolutions if needed — use high-res for the camera facing the most tags, lower-res for others.
- Our code already handles multi-camera fusion in the vision subsystem.

### Verification

1. Place an AprilTag at a known distance (e.g., 3 meters)
2. Open PhotonVision's web interface → verify the tag is detected
3. Check the estimated pose in AdvantageScope — it should match the known position within ~5cm at 3m
4. Walk the robot further away — note the distance where detection drops off
5. Try rapid rotation — if tags blur out, reduce exposure

> **📋 Arena lighting varies wildly.** Bring a laptop to your competition venue and be prepared to adjust exposure/gain during practice matches.

---

## Elastic Dashboard for Matches

[Elastic](https://github.com/Gold872/elastic-dashboard) is a modern FRC dashboard that replaces Shuffleboard. It's faster, prettier, and works well with NetworkTables. Use it for **in-match driver awareness** — not for tuning (use AdvantageScope for that).

### Recommended Match Layout

Build a single, uncluttered tab with only what the driver and operator need to see during a match:

#### Top Row — Critical Status
| Widget | NetworkTables Key | Why |
|--------|-------------------|-----|
| **Match Timer** | `MatchTime` | Countdown — the most important number |
| **Alliance Color** | FMS data (auto-detected) | Red/blue indicator so autonomous knows which side |
| **Robot State** | FMS data (auto-detected) | Disabled / Auto / Teleop / Test |
| **Battery Voltage** | `Battery/Voltage` | Flash red below 11.5V — brownout is imminent |
| **Hub Shift Active** | `HubShift/ShiftedActive` | Green when our alliance can score in the hub |
| **Shift Timer** | `HubShift/ShiftedRemainingTime` | Seconds until shift changes |

#### Middle Row — Subsystem Status
| Widget | NetworkTables Key | Why |
|--------|-------------------|-----|
| **Flywheel Spun Up** | `Flywheel/IsSpunUp` (boolean) | Green = ready to fire. Wire as boolean indicator. |
| **Flywheel RPM** | `Flywheel/FX/VelocityRPM` | Numeric readout for the operator |
| **Turret Angle** | (YAMS auto-publishes) | Confirms turret is tracking the target |
| **Hood Angle** | `Hood/FX/PositionDegrees` | Confirms correct shot angle |
| **Shoot Mode** | `ShooterTelemetry/shootMode` | FULL / STATIC_DISTANCE / FULL_STATIC |

#### Bottom Row — Drive Info
| Widget | NetworkTables Key | Why |
|--------|-------------------|-----|
| **Field View** (2D) | `Drive/Pose` (logged by AdvantageKit) | Mini field map showing robot position — helps verify auto worked |
| **Vision Target** | `Vision/Summary/HasTarget` (boolean) | Green/red indicator for whether cameras see AprilTags |
| **Vision Tag Count** | `Vision/Summary/TagCount` | Number of tags seen this cycle |
| **Hub Distance** | `ShooterTelemetry/hubDistanceMeters` | Distance to hub — only updates when aiming |

### Design Principles

1. **Less is more.** Don't put anything on the dashboard that the driver won't look at during a match. If they need to look at the robot to know something, don't duplicate it on screen.
2. **Use color.** Green = good, Red = bad, Yellow = warning. Boolean indicators are better than numbers for at-a-glance status.
3. **Big text.** The driver is 10+ feet away and stressed. Tiny numbers are useless.
4. **Test it.** Set up the dashboard and have the driver stand at their normal distance. If they can't read it, make it bigger or remove it.

### Setup

1. **Download Elastic** from the [GitHub releases page](https://github.com/Gold872/elastic-dashboard/releases)
2. **Connect** to the robot (it auto-discovers NetworkTables)
3. **Drag widgets** from the sidebar onto the canvas — right-click to configure source paths
4. **Save the layout** as a `.json` file so it persists between sessions
5. **Lock the layout** before competition so no one accidentally moves widgets

> **💡 Pro tip:** Create two tabs: (1) **Match** — the minimal layout above, and (2) **Debug** — a more detailed view with raw mechanism states, motor temperatures, and CAN bus utilization. Only show the Match tab during competition.

---

## Quick Reference Table

| Mechanism | Motor(s) | Control | FF Terms | PID | Motion Profile |
|-----------|----------|---------|----------|-----|---------------|
| Drive (steer) | TalonFX | Position | kS, kV | kP, kD | MotionMagicExpo |
| Drive (drive) | TalonFX | Velocity | kS, kV | kP | — |
| Turret | NEO (27:1) | Position+MP | kS, kV, kA | kP | 1000°/s, 7200°/s² |
| Hood | TalonFX (30:1) | Position+MP | kS, kV | kP | 270°/s, 270°/s² |
| Flywheel | 2× TalonFX | Velocity | kS, kV | kP | — |
| Kicker | Vortex | Velocity (FF-only) | kS, kV | — | — |
| Spindexer | NEO (5:1) | Velocity | kS, kV | kP | — |
| Intake Rollers | Kraken X60 (2:1) | Velocity | kS, kV | kP | — |
| Intake Pivot | 2× Vortex (30:1) | Position+MP | kG, kS | ⚠️ kP=0 | 90°/s, 90°/s² |

### Priority Order for Monday

> **All gains were set with limited testing time. Plan to re-tune everything.**

| Priority | What to do |
|:--------:|-----------|
| 0 | **Weigh the robot** (with bumpers + battery). Update mass in `Drive.java`, `settings.json`, `Constants.DriveConstants` |
| 1 | **Drivetrain Steps 0–6** — everything else depends on accurate drive/odometry |
| 2 | **Update PathPlanner values** (Step 8) — verify `settings.json` matches |
| 3 | **Turret** — needs to track accurately for shooting |
| 4 | **Flywheel** — RPM accuracy affects shot consistency |
| 5 | **Hood** — verify existing gains with the LUT distances |
| 6 | **Intake Pivot** — uncomment FF, tune kG, then add PID |
| 7 | **Spindexer / Kicker / Rollers** — verify, adjust if feeding is inconsistent |
| 8 | **PathPlanner PID** — last, after drive is solid |
