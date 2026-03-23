# PID / Feedforward / Motion Profile Tuning Guide

## Why Tuning Matters (read this first!)

**For mentors and build students who don't write code:** Tuning is calibrating the robot's software to your *specific* hardware. Every motor has "gains" — numbers that control how aggressively the software reaches a target. Too low → sluggish. Too high → overshoots and oscillates. **The right gains make mechanisms snap to target, hold steady, and not shake.**

**Why can't we just calculate the numbers?** Every robot is different — friction, weight, belt tension, carpet texture all matter. Two identical BOMs need different gains. You have to put the robot on the ground and tune it.

**What does it look like?** Someone at a laptop with AdvantageScope while someone watches the robot. Change a number, run the mechanism, check the graph, repeat. A well-tuned mechanism draws a clean square wave. A bad one draws wobbly curves. It's visual and intuitive once you see it.

**Time budget:** 2–4 hours for drivetrain (Steps 0–6), 30–60 minutes per additional mechanism.

---

Tune each subsystem in order — later ones depend on earlier ones being stable.

---

## Table of Contents

1. [General Principles](#general-principles)
2. [YAMS Telemetry Verbosity Levels](#yams-telemetry-verbosity-levels)
3. [Reference Team Swerve Comparison](#reference-team-swerve-comparison)
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

**New to PID?** Work through these in order before touching the real robot:

| # | Resource | What You'll Learn | Link |
|---|----------|-------------------|------|
| 1 | **Mantik: PID Control** | PID and feedforward in plain English | [mantik.netlify.app/…/frc-pid-control](https://mantik.netlify.app/frc/frc-pid-control) |
| 2 | **Mantik: PID Practice — Setup** | YAMS sim + AdvantageScope setup | [mantik.netlify.app/…/pid-tuning-practice-setup](https://mantik.netlify.app/frc/pid-tuning-practice-setup) |
| 3 | **Mantik: PID Practice — Elevator** | Sim practice: find kG, then kP (doubling method) | [mantik.netlify.app/…/pid-tuning-practice-elevator](https://mantik.netlify.app/frc/pid-tuning-practice-elevator) |
| 4 | **Mantik: PID Practice — Arm** | Sim practice: precise kG for arms, motion profiles | [mantik.netlify.app/…/pid-tuning-practice-arm](https://mantik.netlify.app/frc/pid-tuning-practice-arm) |
| 5 | **Mantik: Trapezoidal Profiling** | Motion profiles: max velocity/acceleration | [mantik.netlify.app/…/trapezoidal-motion-profiling](https://mantik.netlify.app/frc/trapezoidal-motion-profiling) |
| 6 | **Mantik: Exponential Profiling** | Advanced profiles for arms (intake pivot, steer) | [mantik.netlify.app/…/exponential-motion-profiling](https://mantik.netlify.app/frc/exponential-motion-profiling) |
| — | **Mantik: AdvantageScope** | Graphing/logging tool | [mantik.netlify.app/…/advantagescope](https://mantik.netlify.app/frc/advantagescope) |
| — | **Mantik: Elastic Dashboard** | Real-time dashboard | [mantik.netlify.app/…/elastic-basics](https://mantik.netlify.app/frc/elastic-basics) |
| — | **YAMS Tuning Docs** | YAMS-specific methodology | [yagsl.gitbook.io/yams](https://yagsl.gitbook.io/yams/) |
| — | **AK Swerve Template Docs** | Drivetrain calibration steps | [docs.advantagekit.org](https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template/) |

> **Recommended path:** PID Control (#1) → Elevator sim (#3) → Arm sim (#4) → come back here. Budget ~1 hour for sim practice.

### Tuning Order (within any mechanism)

Every mechanism follows the same pattern: **feedforward first, then PID.** FF does the heavy lifting (~80–90%), PID cleans up the rest.

> *Feedforward is your aim — it gets the ball most of the way to the target. PID nudges it the last few inches. If your aim (FF) is bad, no amount of nudging (PID) will help.*

**Before you start:**
- Set ALL gains to 0 (kP, kI, kD, kS, kV, kA, kG).
- Set motion profile limits to **safe, low values**.
- Open AdvantageScope with **Position vs Setpoint** on the same plot.

---

**For mechanisms that hold a position against gravity (turret, hood, intake pivot):**

| Step | Gain | What to do | What to look for |
|------|------|------------|-----------------|
| 1 | **kG** | Compensates gravity. Binary search: start low, double until it drifts up, then narrow. For arms, get to **2–3 decimal places.** | Position line is **flat** — not drifting. |
| 2 | **kV** | Voltage per unit speed. Start at 0.1, increase in big jumps. | Position **slope matches** reference during cruise. |
| 3 | **kA** | Handles acceleration. Start at 0.001, increase carefully. | Position **overlaps** reference during accel/decel. |
| 4 | **kP** | Corrects remaining error. **Double from 0.1** until slight overshoot, then back off 10–20%. | Graph looks like a **rectangle** — sharp edges, flat top. |
| 5 | **kD** | Dampens overshoot. Start tiny (0.005–0.05). | Overshoot gone. Jitter/buzzing = too high. |
| 6 | **kI** | **Almost never needed.** Last resort for persistent steady-state error. | Error disappears. ⚠️ Clear integral accumulator first. |

---

**For mechanisms that spin at a speed (flywheel, rollers, spindexer, kicker):**

| Step | Gain | What to do | What to look for |
|------|------|------------|-----------------|
| 1 | **kS** | Slowly increase voltage from 0 until the mechanism *barely* moves. That voltage is kS. | Mechanism just starts to move. *(Skip in sim.)* |
| 2 | **kV** | Set a target speed, adjust kV until actual matches. Test at several speeds. | Velocity line sits on the setpoint line at steady state. |
| 3 | **kP** | Increase until setpoint reached in ~0.5s without oscillation. | Fast spin-up, no wobble. |
| 4 | **kD** | Only if oscillating. Usually leave at 0 for velocity loops. | Oscillation stops. |

---

**How to read the position graph:**

| Graph Shape | Meaning |
|-------------|---------|
| ✅ Rectangular — sharp rise, flat top, sharp fall | Well-tuned |
| ❌ Slow, rounded curve | kP too low or FF insufficient |
| ❌ Overshoots then oscillates | kP too high or profile limits too aggressive |
| ❌ Slowly drifts | kG wrong (gravity) or kS wrong (horizontal) |

> **Pro tip:** When tuning kV/kA, temporarily set kP high to push the mechanism to the right operating point, then zero kP before adjusting FF.

### Motion Profiling: When and Which Type

A motion profile ramps the mechanism smoothly to target instead of slamming to full power, preventing broken gears, skipped belts, and hard-stop impacts.

**Every position-controlled mechanism needs a motion profile.** Velocity mechanisms (flywheels, rollers) don't.

> ⚠️ **YAMS requirement:** For position-controlled mechanisms, `kV` and `kA` feedforward only work when a motion profile is configured. Without one, YAMS has no velocity/acceleration reference to feed forward against — FF terms are silently ignored and only PID does work. **Always set profile limits before tuning FF.**

| Type | How it works | Used by |
|------|-------------|---------|
| **Trapezoidal** | Constant accel → cruise → constant decel. Simple. | Turret, Hood |
| **Exponential** | Accel follows motor voltage-speed curve (smoother). | Drive steer, Intake pivot |

#### Understanding Exponential Profile Gains

Exponential profiles model how a **real DC motor** accelerates — fast at low speed, then tapering off as back-EMF builds. This produces smoother, faster moves than trapezoidal profiles, especially for gravity-loaded mechanisms (arms, pivots) and continuous-rotation mechanisms (swerve steer).

**Trapezoidal vs Exponential — what's different:**

| | Trapezoidal | Exponential |
|---|---|---|
| **Parameters** | `maxVelocity`, `maxAcceleration` | `maxVoltage`, motor model constants `A` and `B` |
| **Acceleration shape** | Constant (flat line) | Varies with speed (exponential curve) |
| **Result** | Trapezoid-shaped velocity | S-curve-shaped velocity |
| **Best for** | Simple point-to-point moves | Arms, pivots, steering — anything where motor physics matter |

**The A and B constants** come from the DC motor state-space model: $\dot{v} = Av + Bu$, where $v$ is velocity and $u$ is voltage. WPILib's `DCMotor` class provides these based on motor specs and gearing. You don't need to calculate them manually — YAMS has helper methods.

**Three ways to configure exponential profiles in YAMS:**

| Method | What you provide | When to use |
|--------|-----------------|-------------|
| `withExponentialProfile(maxVoltage, DCMotor, momentOfInertia)` | Motor type + MOI | **Rotational** mechanisms (arms, pivots). Most common. |
| `withExponentialProfile(maxVoltage, DCMotor, mass, radius)` | Motor type + mass + pulley radius | **Linear** mechanisms (elevators). |
| `withExponentialProfile(maxVoltage, maxVelocity, maxAccelAtStall)` | State-space constraints directly | When you've already measured or calculated A & B. |

**Example (intake pivot):**
```java
// From motor physics — YAMS derives A and B automatically
.withExponentialProfile(Volts.of(12), DCMotor.getNeoVortex(2), kMomentOfInertia)

// Or from measured/calculated state-space values
.withExponentialProfile(Volts.of(12), kMaxAngularVelocity, kMaxAccelAtStall)
```

You can also use an explicit `ExponentialProfilePIDController` for full control:
```java
.withClosedLoopController(new ExponentialProfilePIDController(kP, kI, kD,
    new ExponentialProfile.Constraints(maxVoltage, maxVelocity, maxAccelAtStall)))
```

**How YAMS runs exponential profiles on different motor controllers:**

| Motor Controller | Where profile runs | How it works |
|-----------------|-------------------|-------------|
| **TalonFX** (Kraken, Falcon) | **Hardware** (firmware) | YAMS converts A/B → `MotionMagicExpo_kV` and `MotionMagicExpo_kA` and sends a `MotionMagicExpoVoltage` request. All computation runs on the motor controller at ~1 kHz. |
| **SparkMax / SparkFlex** | **RIO** (software) | YAMS runs the `ExponentialProfile` in a dedicated `Notifier` thread on the roboRIO. Each tick it calculates the next profile state, runs PID + FF, and sends a voltage command. Prints `"====== Spark(X) Using RIO Closed Loop Controller ======"` at startup. |

> **Key insight:** YAMS supports exponential profiles regardless of motor controller. TalonFX gets hardware acceleration; Spark gets software-equivalent behavior. The API is identical — only the underlying execution differs.

**TalonFX hardware conversion formulas:**
- `MotionMagicExpo_kV = -A / B` (converts state-space constants to CTRE's velocity feedforward)
- `MotionMagicExpo_kA = 1.0 / B` (converts to CTRE's acceleration feedforward)

> These are set automatically by YAMS — you don't need to compute them. They're documented here so you understand what's happening in Phoenix Tuner X if you inspect the config.

**When to use exponential vs trapezoidal:**

| Use exponential when… | Use trapezoidal when… |
|----------------------|----------------------|
| Mechanism fights gravity (arms, pivots) | Simple horizontal moves (turret) |
| You want the smoothest possible motion | Mechanism is well-damped and simple |
| Motor is near its performance limits | You want the simplest tuning |
| Mechanism has high inertia | Short travel range (hood: 0–30°) |

#### Recommended Starting Values

These are conservative starting points. All values can be tuned live via YAMS NT entries. Start here, then increase toward the theoretical maxes using the tuning procedure below.

| Mechanism | Motor / Gear Ratio | Theoretical Max Speed | **Starting Cruise Velocity** | **Starting Max Acceleration** | Code Location |
|-----------|-------------------|----------------------|------------------------------|-------------------------------|---------------|
| **Turret** | NEO 5676 RPM / 27:1 | ~1261°/s | **1000°/s** | **7200°/s²** | `ShooterConstants.TurretConstants` |
| **Hood** | Kraken X44 / 30:1 | ~600°/s | **270°/s** | **270°/s²** | Hardcoded in `HoodSubsystem` constructor |
| **Intake Pivot** | 2× Vortex / 30:1 | ~600°/s | **90°/s** | **90°/s²** | `IntakeConstants.Pivot` |

> **Why these values?**
> - **Turret** is the fastest mechanism — 1000°/s is ~80% of free speed, 7200°/s² is aggressive but the 27:1 gearing provides plenty of torque. If it slams into the hard stop, reduce acceleration first.
> - **Hood** travels only 0–30° total — 270°/s means the full range in ~0.11s. This is conservative. Increase velocity if shot transitions feel slow.
> - **Intake Pivot** uses 90°/s because the pivot is heavy (two motors, 30:1 gearing). 90°/s² is very gentle. Once kG is tuned and it holds position reliably, increase both by 2–3×.

#### How to tune profile limits

([Detailed interactive guide](https://mantik.netlify.app/frc/trapezoidal-motion-profiling))

1. Set acceleration high. Increase max velocity until the mechanism can't keep up. Back off 10–20%.
2. Increase acceleration until overshoot/vibration. Back off 15–25%.
3. Overshoot at corners → reduce acceleration. Motion feels slow → increase both.

### Tools

| Tool | Purpose |
|------|---------|
| **YAMS Live Tuning** | Primary tool. With `TelemetryVerbosity.HIGH`, publishes editable NT entries for all gains, setpoints, and profile limits. Change gains live without redeploying. |
| **AdvantageScope** | Log replay, setpoint vs measured overlays. Import `AdvantageScope Swerve Calibration.json` for drive tuning. |
| **CTRE Tuner X** | Drive/steer gains, MotionMagic config, real-time plots. Works in sim (`localhost`). |
| **REV Hardware Client** | Encoder verification, absolute encoder offsets, firmware updates. |
| **WPILib SysId** | Automated FF characterization (quasistatic + dynamic). Wired for drivetrain in `setupSysid()`. |
| **Elastic Dashboard** | Real-time telemetry graphs, boolean indicators, YAMS tuning widgets. |

### YAMS Live Tuning (how it works)

Change gains on the fly without redeploying — the fastest way to tune.

1. Switch Driver Station to **Test** mode, enable the robot.
2. In Elastic, find `SmartDashboard > MECHANISM_NAME > Commands > Live Tuning`. Drag to dashboard and click.
3. Edit gains in the NT entries — they take effect immediately.
4. Watch AdvantageScope to see the result.

**Editable entries** (all YAMS mechanisms when `tuningMode = true`):

| Entry | Units | Notes |
|-------|-------|-------|
| kP, kI, kD | — | PID gains |
| kS, kV, kA, kG | volts / volts per (unit/s) | Feedforward terms |
| Setpoint position | degrees | Target position |
| Setpoint velocity | RPM | Target speed |
| Max velocity / acceleration | RPM, RPM/s | Motion profile limits |

> ⚠️ Always test in sim first. Typing kP=300 when you meant 3.0 causes violent oscillation.

> **Where:** Telemetry under `NT:/Mechanisms`. Live Tuning commands under `NT:/SmartDashboard`.

### AdvantageScope Tips

- **Key graph:** Drag both `angle` (or `velocity`) and `setpoint` onto the same line graph. The gap tells you everything.
- **Convert units:** Right-click axis → "Edit Axis" → multiplier 360 (rot → deg) or 60 (RPS → RPM).
- **Mark trials:** Press `N` to note "kP=3.0" so you can compare runs later.
- **Check saturation:** Plot voltage on a second Y-axis. If pegged at ±12V, the motor is maxed out.
- **Export:** Select time range → right-click → Export to CSV.

### Elastic Dashboard Tips

- **"At setpoint" indicator:** Add a boolean widget for `Mechanisms/HoodIsMoving` etc. — turns green when on target.
- **Group widgets:** Create a "Tuning" tab — gains left, graph right, current draw below.
- **Quick setpoints:** Use tunable number widgets to jump between test positions without code changes.

### Phoenix Tuner X Tips (CTRE motors)

- **Position vs Reference overlay:** Plot `Position` and `ClosedLoopReference` together. The gap shows tracking quality.
- **Useful signals:** `Position`, `ClosedLoopReference`, `ClosedLoopOutput`, `StatorCurrent` — all on one graph.
- **Sim:** Connect to `localhost` for the same plots on simulated motors.

### Position vs Reference: The Key Graph for kV/kA Tuning

When tuning kV and kA, you need two lines on the same graph:
- **Position** (or velocity) — where the mechanism *actually* is
- **Reference** — where the motion profile says it *should* be right now

This is NOT the final setpoint — the reference changes every cycle as the profile ramps up, cruises, and ramps down.

**kV:** During cruise (straight part), the two lines should have the **same slope**. Position less steep → kV too low.

**kA:** During accel/decel (curved parts), the two lines should **overlap**. Position lags during accel → kA too low.

#### CTRE mechanisms (Hood, Flywheel) — ✅ Set up

CTRE motors provide the reference signal via `getClosedLoopReference()`. We already log it:

| Mechanism | Position | Reference |
|---|---|---|
| **Hood** | `Hood/FX/PositionDegrees` | `Hood/FX/ReferenceDegrees` |
| **Flywheel** | `Flywheel/FX/VelocityRPM` | `Flywheel/FX/ReferenceRPM` |

> For future CTRE mechanisms, follow the pattern in `HoodSubsystem.java` / `FlywheelSubsystem.java`.

#### REV mechanisms (Turret, Spindexer, Kicker) — ⚠️ No reference signal

REV SparkMax/SparkFlex doesn't expose the MAXMotion reference trajectory.

**Workaround:**
1. **kV:** Watch position during cruise — should move at the profile's max velocity. Slower → increase kV.
2. **kA:** Watch acceleration shape. With kA=0, transitions are soft. Increase kA until they're crisp.

### Safety Checklist (before every session)

- [ ] Robot on blocks (wheels off ground) for drive tuning
- [ ] Current limits set (already done for all subsystems)
- [ ] Soft limits enabled in YAMS configs
- [ ] Someone on the E-stop
- [ ] `tuningMode = true` in `Constants.java`

> `tuningMode = true` sets telemetry to HIGH. At competition, set `false` for MID. See [YAMS Telemetry Verbosity Levels](#yams-telemetry-verbosity-levels).

---

## YAMS Telemetry Verbosity Levels

| Level | Publishes | When to use |
|-------|-----------|-------------|
| **LOW** | Setpoints, sensor readings, position/velocity | Match play (minimal overhead) |
| **MID** | + Voltage, current, raw rotor data | Competition (debug brownouts post-match) |
| **HIGH** | + All tunable gains, limits, temps, profile params — all editable live | Shop tuning and practice |

`Constants.telemetryVerbosity()` returns HIGH when `tuningMode = true`, MID when `false`. At HIGH, each motor publishes ~30+ NT entries/cycle. With 10+ motors that's 300+ entries — switch to MID at competition to reduce loop time.

| Situation | Level | `tuningMode` |
|-----------|-------|:---:|
| Shop / practice | HIGH | `true` |
| Competition matches | MID | `false` |

---

## Reference Team Swerve Comparison

How our swerve constants compare to other FRC teams (**2026 season**). All use TalonFX (Kraken) swerve with Phoenix 6 unless noted.

> **Sources:** Public GitHub repos: [6328](https://github.com/Mechanical-Advantage/RobotCode2026Public), [LASA PH](https://github.com/lasarobotics/PH2026), [LASA PR](https://github.com/lasarobotics/PR2026), [WHS 3467](https://github.com/WHS-FRC-3467/Skip-5.16-Platypus), [Hammerheads 5000](https://github.com/hammerheads5000/2026Rebuilt), [Lynk 9496](https://github.com/LynkRobotics/RobotCode2026Public).

### Hardware Summary

| Team | Module | Drive Ratio | Steer Ratio | Wheel Radius | Track/Wheelbase |
|------|--------|:-----------:|:-----------:|:------------:|:---------------:|
| **3216 (us)** | WCP SwerveX2ST (X2, 18T) | 4.667:1 | 25.9:1 | **1.80"** | 23" sq |
| 6328 Darwin | SDS MK5i R1 | 7.03:1 | 26.0:1 | 1.996" | — |
| 6328 Alphabot | SDS MK4i L3 | 6.12:1 | 21.43:1 | 1.880" | — |
| LASA (PH/PR) | — | 6.03:1 | 26.09:1 | 2.00" | — |
| WHS 3467 | — | 6.0:1 | 24.0:1 | 1.97" | 22" sq |
| Hammerheads 5000 | — | 6.03:1 | 26.09:1 | 1.985" | 22.5 × 20.75" |
| Lynk 9496 | SDS MK5n R2 | ~6.03:1 | 26.09:1 | ~2.0" | 20.75" sq |

**Takeaway:** Most teams use 6.0–6.12 drive ratios. Our 4.667:1 ([WCP X2 + 18T](https://docs.wcproducts.com/welcome/gearboxes/wcp-swerve-x2s)) is significantly faster-geared — higher top speed, less torque per amp. Re-verify wheel radius with [Step 3](#step-3-wheel-radius-characterization).

### Current Limits

| Team | Drive Stator | Steer Stator | Slip Current |
|------|:-----------:|:-----------:|:------------:|
| **3216 (us)** | **80A** | **60A** | **120A** |
| 6328 | 80A | 40A | — (uses TorqueFOC) |
| LASA (PH/PR) | — | 60A | 120A |
| WHS 3467 | — | 60A | 94A |
| Hammerheads 5000 | 80A | 60A | 95A |
| Lynk 9496 | — | — | — |

**Takeaway:** 80A drive / 60A steer is the consensus. Our 120A slip current matches LASA but is higher than most (~95A). Run the [slip test](#step-6-slip-current-measurement) to find our actual value.

### Drive/Steer Gains — Voltage Mode

Apples-to-apples comparison of teams using Voltage mode. TorqueCurrentFOC teams have different gain scales — see next table.

| Team | Steer kP | Steer kD | Steer kS | Steer kV | Drive kP | Drive kS | Drive kV |
|------|:-------:|:-------:|:-------:|:-------:|:-------:|:-------:|:-------:|
| **3216 (us)** | **100** | **0.5** | **0.1** | **2.48** | **0.1** | **0.26** | **0.585** |
| LASA (PH/PR) | 100 | 0.5 | 0.1 | 2.49 | 0.1 | 0 | 0.124 |
| Hammerheads (generated) | 100 | 0.5 | 0.1 | 2.49 | 0.1 | 0 | 0.124 |
| Hammerheads (tuned) | 1000 | 8 | 6 | 0 | 2 | 0.237 | 0.733 |

> **Note:** Hammerheads has both generated `TunerConstants` (matching LASA — likely Tuner X defaults) and hand-tuned `SwerveConstants` with higher steer gains (TorqueCurrentFOC steer, Voltage drive).

**Takeaway:** Steer kP=100 / kD=0.5 / kS=0.1 / kV≈2.49 is the Voltage-mode consensus. Our steer matches. Our drive kV (0.585) is higher than LASA (0.124) — expected with our lower gear ratio (4.667 vs 6.03); kV scales inversely with ratio. **Re-run drive FF after wheel radius characterization.**

### Drive/Steer Gains — TorqueCurrentFOC Mode

Gains in **amps**, not volts. Do not mix with Voltage mode gains.

| Team | Steer kP | Steer kD | Drive kP | Drive kS |
|------|:-------:|:-------:|:-------:|:-------:|
| 6328 Darwin | 4000 | 50 | 35 | 5.0 |
| WHS 3467 | 2000 | 20 | 60 | 7.26 |
| Hammerheads (steer only) | 1000 | 8 | — | — |

**Takeaway:** TorqueFOC gains are 10–40× larger. If we switch, 6328 and WHS 3467 are good starting points. See [TorqueFOC section](#torquefoc--available-upgrade-path).

### Control Mode

| Team | Steer Output | Drive Output | Phoenix Pro? |
|------|:-----------:|:-----------:|:---:|
| **3216 (us)** | **Voltage** | **Voltage** | **Yes** |
| 6328 | TorqueCurrentFOC | TorqueCurrentFOC | Yes |
| LASA (PH/PR) | Voltage | Voltage | — |
| WHS 3467 | TorqueCurrentFOC | TorqueCurrentFOC | Yes |
| Hammerheads | TorqueCurrentFOC | Voltage | Yes |
| Lynk 9496 | (custom) | (custom) | — |

**Voltage mode is the most common.** 6328 and WHS 3467 use full TorqueFOC. Hammerheads splits (TorqueFOC steer + Voltage drive). We can switch later — see [TorqueFOC section](#torquefoc--available-upgrade-path).

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

**Takeaway:** Measured speeds are typically 10–30% below theoretical. Our 6.02 m/s will likely measure ~5.0–5.5 m/s. Run the [max speed test](#step-5-max-speed-measurement).

### Pigeon 2 Configuration

| Team | MountPose Configured? | Roll | Pitch | Yaw |
|------|:--------------------:|:----:|:-----:|:---:|
| **3216 (us)** | **No** | — | — | — |
| LASA PR2026 | ✅ | 179.96° | 1.14° | -90.45° |
| WHS 3467 | Partial | 180° | — | — |
| Hammerheads | ✅ | — | — | -90° |
| Others | `null` (skipped) | — | — | — |

**Takeaway:** If our Pigeon isn't flat + forward-facing, configure `MountPose`. See [Pigeon IMU Calibration](#pigeon-imu-calibration).

### Interesting Patterns From Reference Teams

| Pattern | Who | What It Does |
|---------|-----|-------------|
| **LoggedTunableNumber** | 6328, WHS, Hammerheads | Runtime-tunable gains with `ifChanged()` — only applies when values change. More boilerplate than YAMS. |
| **WheelSlipAuto** | WHS 3467 | Auto routine that ramps voltage logging current vs velocity for slip characterization. |
| **Separate sim constants** | WHS, Hammerheads | Different kP/kV for sim vs real. Could improve our sim accuracy. |
| **SwerveSetpointGenerator** | BroncBotz 3481 | 254's port — limits per-module acceleration for smoother motion. |
| **Wheel COF** | 6328 (1.5), Hammerheads (2.255) | PathPlanner traction model. Our 1.2 is conservative. |
| **Open loop ramps** | LASA (0.5s), Lynk (0.25s) | Voltage ramp to reduce wheel slip on acceleration. |

### Shooter Mechanism Gains (Reference)

Several reference teams in 2026 also run flywheel + hood shooters. Use this as a sanity check when tuning our gains.

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

**Takeaway:** Our flywheel kP (0.2) is low — try 0.5–1.0 if recovery is slow after a shot. Hood kP (300) is reasonable for Voltage mode. Kicker FF-only (kP=0) is common — only add PID if it stalls on ball contact.

> **Action items:** (1) Measure slip current, (2) Configure Pigeon MountPose if needed, (3) Consider open loop ramps (0.25s), (4) Measure wheel COF.

---

## TorqueFOC — Available Upgrade Path

> **Not needed now.** Get Voltage mode working first. TorqueFOC is a future upgrade.

TorqueFOC (TorqueCurrentFOC) commands **current** (amps) instead of voltage, giving consistent performance as the battery drains and more predictable traction control. **We have Phoenix Pro and the code is wired for it** — switching is a one-line change, but requires a complete retune.

### Why (later)

- Consistent performance as battery sags (12V → 11V)
- Better traction control (torque directly limited)
- Used by 6328; most others use Voltage

### What changes

| | Voltage (current) | TorqueFOC |
|---|---|---|
| Steer kP | 100 | ~4000 |
| Steer kD | 0.5 | ~50 |
| Drive kP | 0.1 | ~35 |
| kS/kV units | Volts | Amps |

All gains change dramatically — you cannot reuse Voltage-mode gains.

### How to switch

1. In `TunerConstants.java`, change both output types to `ClosedLoopOutputType.TorqueCurrentFOC`
2. Zero all gains and start fresh
3. Re-run entire drivetrain calibration (Steps 1–7)
4. Optionally add torque ramp (0.02s) to prevent current spikes

### Motion Profile Types by Mechanism

| Mechanism | Profile Type | Why |
|-----------|-------------|-----|
| Drive (steer) | **MotionMagicExpo** (firmware) | Smooth continuous steering |
| Turret | **MAXMotion trapezoidal** (firmware) | Simple point-to-point |
| Hood | **MotionMagic trapezoidal** (firmware) | Small mechanism, short moves |
| Intake Pivot | **ExponentialProfile** (RIO) | Gravity-aware — accelerates faster going down |

> **Rule of thumb:** Trapezoidal = constant acceleration. Exponential = acceleration varies with speed. Use exponential for steering and gravity-loaded arms.

---

## YAMS FF + Motion Profile Requirement

> **Key rule:** For position-controlled mechanisms, **kV and kA only work if a motion profile is configured.**

kV multiplies the *velocity the profile says the mechanism should be moving right now*. No profile → no planned velocity → kV × 0 = 0.

**All our positional mechanisms already have profiles:**

| Mechanism | FF | Motion Profile | ✅ |
|-----------|:--:|:--------------:|:--:|
| Turret | kS=0, kV=1.0, kA=0.05 | 1000°/s, 7200°/s² | ✅ |
| Hood | kS=0.45, kV=3.0 | 270°/s, 270°/s² | ✅ |
| Intake Pivot | kG=0.21, kS=0.11 | 90°/s, 90°/s² | ✅ |

**If you add FF to a new positional mechanism, always pair it with a motion profile.**

Velocity mechanisms (flywheel, kicker, spindexer, rollers) are unaffected — kV multiplies the velocity setpoint directly.

---

## YAMS SysId Helpers

YAMS mechanisms have a built-in `sysId()` method that creates a complete characterization routine. Gives starting values for kS, kV, kA, kG.

| Approach | Best for | Use on |
|----------|----------|--------|
| **SysId** | Getting FF gains within ~5% | Intake pivot (kG), flywheel (kV), turret (kV) |
| **Manual** | Quick iteration with a ballpark | Hood, kicker, spindexer |
| **YAMS Live Tuning** | Fine-tuning after SysId | All mechanisms |

- **REV:** YAMS uses duty cycle for cleaner data. REVLib 2026+ auto-writes `.revlog` files for AdvantageScope.
- **CTRE:** YAMS uses `VoltageOut`. Start/stop Signal Logger manually (bind to buttons). Extract `.hoot` logs via Tuner X.

---

## 1. Drivetrain (CTRE Swerve)

**File:** `TunerConstants.java` (generated by Tuner X — don't hand-edit gains)
**Motors:** 8× TalonFX (4 drive + 4 steer), CAN FD at 250 Hz odometry
**Control:** Phoenix 6 Voltage mode (TorqueFOC available — see [upgrade path](#torquefoc--available-upgrade-path))

> ⚠️ **AK uses different gain scales than CTRE's default swerve code.** AK applies gear ratio in firmware, so FF/PID from other projects won't work directly. Use the AK characterization routines.

> Import `AdvantageScope Swerve Calibration.json` for predefined tuning tabs.

### Enabling Characterization Routines

1. Uncomment `setupSysid();` in `RobotContainer` constructor (line 224)
2. Deploy and reboot — auto chooser shows characterization options
3. **Re-comment before competition**

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

| Measurement | How | Value | Status |
|---|---|---|---|
| **Robot mass** (bumpers + battery) | Scale | 63.5 kg (140 lbs) | ✅ Verify on scale |
| **Wheel radius** (effective) | [Step 3](#step-3-wheel-radius-characterization) | 1.8" (nominal) | ⚠️ Characterize on carpet |
| **Max speed** | [Step 5](#step-5-max-speed-measurement) | 6.02 m/s (theoretical) | ⚠️ Measure |
| **Slip current** | [Step 6](#step-6-slip-current-measurement) | 120A (default) | ⚠️ Wall test |
| **Robot MOI** | PathPlanner GUI | 6.883 kg⋅m² | ⚠️ Verify with auto paths |

> **Keep in sync:** `Drive.java`, `settings.json` (PathPlanner), and `Constants.DriveConstants`.

### Step 0: Pre-flight Checks

- [ ] Phoenix Tuner X connected — all 8 TalonFX + 4 CANcoders green
- [ ] Module offsets correct — wheels point straight forward
- [ ] Motor inversions correct — forward = positive velocity, CCW steer = positive position
- [ ] CAN bus healthy — 0% error rate, CANFD bus
- [ ] Firmware updated (Phoenix 6 v26)
- [ ] Robot on blocks for Steps 0–2, 4. Ground for Steps 3, 5, 6.

### Step 1: Steer PID

Gains: `kP=100, kI=0, kD=0.5, kS=0.1, kV=2.48, kA=0` · MotionMagicExpo (runs in firmware)

**Tune on blocks:**

1. **kS:** Command slow steer rotation. Increase until module barely moves. (0.1)
2. **kV:** Command moderate speed. `kV = (voltage - kS) / velocity`. Match slopes during cruise. (2.48)
3. **kP:** Command 0°→90°. Start ~50, increase until settling <50ms. (100 — consensus across reference teams)
4. **kD:** If overshoot, add 0.1–1.0. (0.5)
5. **kI:** Leave at 0.

**Verify:** Plot `SwerveStates/Measured` vs `SwerveStates/SetpointsOptimized` — angles should track with no oscillation.

### Step 2: Drive Feedforward (kS, kV)

Gains: `kP=0.1, kI=0, kD=0, kS=0.26, kV=0.585`

> Use the AK characterization routine — don't copy FF from non-AK projects.

**Quick method — "Drive Simple FF Characterization" auto:**

1. Open space, wheels on ground, fully charged battery
2. Select "Drive Simple FF Characterization" from auto chooser
3. Enable — robot drives forward with slow ramp
4. Run 5–10 seconds, disable
5. Read kS and kV from Driver Station console
6. Update `driveGains` in `TunerConstants.java`
7. Repeat 2–3 times and average

> **Expected:** kV ≈ 0.4–0.7, kS ≈ 0.1–0.4 for Kraken X60 with 4.667:1 ratio.

**Full SysId** (also gets kA): Run four routines (quasistatic/dynamic, forward/reverse), analyze with WPILib SysId tool. AK logs positions in radians.

### Step 3: Wheel Radius Characterization

Value: `kWheelRadius = 1.8"` (nominal — needs characterization)

1. Place robot **on carpet** (not hard floor)
2. Select "Drive Wheel Radius Characterization" from auto chooser
3. Enable — robot slowly rotates in place
4. Run 2–3 full rotations, disable
5. Read measured radius from DS console
6. Update `kWheelRadius` in `TunerConstants.java`

> **Expected:** ~1.70–1.85". Re-run when swapping wheels or changing carpet.

### Step 4: Drive PID (kP)

After FF and wheel radius are set:

1. In teleop, do a sudden joystick push (0 → ~2 m/s)
2. Plot measured vs setpoint velocities in AdvantageScope
3. Increase kP until rise time <100ms with no oscillation (try 0.05, 0.1, 0.2…)
4. kD: usually 0 for velocity loops
5. Current kP=0.1 is a good starting point

> Oscillating → reduce kP. Steady-state error → FF problem (kV), not PID.

### Step 5: Max Speed Measurement

Value: `kSpeedAt12Volts = 6.02 m/s` (theoretical — measure it)

This normalizes joystick input. Too high → robot never reaches full speed. Too low → joystick saturates early.

1. Open space (30+ ft), fully charged battery
2. Drive full speed forward, hold 3+ seconds until velocity stabilizes
3. Read peak sustained velocity in AdvantageScope (plateau, not spike)
4. Update `kSpeedAt12Volts` in `TunerConstants.java` and `maxDriveSpeed` in `settings.json`

> **Expected:** ~5.5–5.9 m/s (friction, motor efficiency, and battery sag reduce from theoretical).

### Step 6: Slip Current Measurement

Value: `kSlipCurrent = 120A` (reference teams use 80–95A — verify)

Slip current = stator current where wheels lose traction. Used for traction control.

1. Push robot against a **solid wall**, all wheels on carpet
2. Plot drive current and velocity for one module in AdvantageScope
3. Slowly push joystick 0% → 100% (into wall)
4. Watch velocity: ~0 (gripping) → **sudden jump** (slipping)
5. Current at that moment = slip current
6. Repeat 2–3 times, average, update `kSlipCurrent`

> **Expected:** 60–100A for Kraken X60 on carpet. Most reference teams use 80–95A.

> **Note:** If `kSlipCurrent` (120A) exceeds the stator limit (80A), traction control never activates. Ensure the measured value makes sense relative to your current limit.

### Step 7: Odometry Frequency

**No action needed.** CAN FD runs odometry at 250 Hz (set in `Drive.java`). Only reduce to 200 Hz if you see loop overruns.

### Step 8: PathPlanner Configuration

PathPlanner needs accurate physical parameters in **two places that must match:**

| Parameter | `Drive.java` | `settings.json` | Status |
|---|---|---|---|
| Robot mass | 63.503 kg | 63.503 | ✅ |
| Robot MOI | 6.883 kg⋅m² | 6.883 | ⚠️ Verify with autos |
| Wheel COF | 1.2 | 1.2 | ✅ |
| Wheel radius | 0.04572 m | 0.04572 | ⚠️ Step 3 |
| Max speed | 6.02 m/s | 6.02 | ⚠️ Step 5 |
| Drive current limit | 80A | 80A | ✅ |

**PID** (`Constants.PathPlannerConstants`): Translation kP=5.0, Rotation kP=5.0. Reference: LASA (3.0/4.0), Hammerheads (3.0/2.0), BroncBotz (5.0/5.0), 6328 Choreo (8.0/4.0). Our 5.0/5.0 is aggressive — reduce to 3.0 if paths overshoot.

1. **Tune drivetrain first** — PP PID sits on top of drive gains
2. **Translation kP:** Straight path. Drifts → increase. Oscillates → decrease.
3. **Rotation kP:** Path with heading changes. Sluggish → increase. Oscillates → decrease.

**MOI:** Auto paths overshoot rotations → MOI too low. Sluggish → too high.

### Advanced: Swerve Setpoint Generator

> Not needed now. Steer already uses MotionMagicExpo. 254's setpoint generator is available in PathPlanner but not enabled — consider after basic calibration.

---

## 2. Turret

**Motor:** SparkMax + NEO (27:1) · **Control:** YAMS Pivot + MAXMotion · **FF:** kS, kV, kA
**File:** `ShooterConstants.TurretConstants` (gains), `TurretSubsystem.java` (YAMS config)

Gains: `kP=3.0, kD=0, kS=0, kV=1.0, kA=0.05` · Profile: `1000°/s, 7200°/s²`

> ⚠️ Profile reduced from 1440°/s — NEO through 27:1 maxes out at ~1261°/s.

The turret is a **horizontal** position-controlled mechanism — no gravity compensation needed (`kG=0`). It uses MAXMotion trapezoidal profiling on the SparkMax. The turret tracks a target angle from the look-up table (LUT) and vision, so it needs to be fast and precise.

### Pre-flight

- [ ] Turret moves freely by hand with power off (no binding, smooth rotation)
- [ ] Absolute encoder reads 0° when turret faces forward (verify in REV Hardware Client)
- [ ] Soft limits set in YAMS config (prevent cable wrap or hard-stop damage)
- [ ] `tuningMode = true` in `Constants.java`
- [ ] Open AdvantageScope with `Shooter/Turret/angle` and `Shooter/Turret/setpoint` on the same line graph

### Step 1: kS — Static Friction

The turret is horizontal, so there's no gravity component. kS compensates for friction in the gearing.

1. Set all gains to 0 (kP, kD, kS, kV, kA)
2. In Test mode with YAMS Live Tuning, slowly increase voltage from 0
3. Record the voltage where the turret **just barely** starts to move
4. That voltage is kS
5. Verify: turret should start moving smoothly from any position, in both directions

> **Expected:** kS ≈ 0–0.3V for NEO through 27:1. Our current kS=0 suggests friction is minimal. If the turret hesitates at move start, increase kS.

### Step 2: kV — Velocity Feedforward

kV converts the profile's planned velocity into the voltage needed to sustain that speed.

1. Set kP=0, kS to the value from Step 1, kV=0.5 (starting guess)
2. Command a large move (e.g., 0°→180°)
3. In AdvantageScope, overlay `angle` and `setpoint` — zoom into the **cruise** (straight) portion
4. If actual velocity during cruise is **lower** than the profile velocity → increase kV
5. If actual velocity during cruise is **higher** → decrease kV
6. Iterate until the slopes match during cruise

> **Verification:** During the cruise segment, the position line should be parallel to (and ideally overlap) the reference line. A consistent gap means kV is close but kS may be off.

> **REV note:** SparkMax doesn't expose the MAXMotion reference trajectory. Compare the actual velocity (derivative of position) against the profile's configured max velocity instead.

### Step 3: kA — Acceleration Feedforward

kA converts the profile's planned acceleration into voltage.

1. Keep kS and kV from above, set kA=0.001
2. Command a large move and zoom into the **curved** acceleration/deceleration portions
3. If position **lags** the reference during acceleration → increase kA
4. If position **leads** the reference during acceleration → decrease kA
5. Typical range: 0.01–0.1

> **Expected:** kA=0.05 is a reasonable starting point. The 27:1 gearing provides significant torque so the acceleration error without kA may be small.

### Step 4: kP — Proportional Gain

1. Set kP=0.5 (start low)
2. Command 0°→90° and watch the settle time
3. Double kP until settle time <200ms: 0.5 → 1.0 → 2.0 → 4.0
4. If it overshoots, back off 10–20%
5. Target: **crisp rectangular** position graph (sharp rise, flat top, sharp fall)

> **Expected:** kP=3.0 should give fast settling. If the turret oscillates at 3.0, check that kV is correct first — bad FF makes PID compensate too hard.

### Step 5: kD — Derivative Gain

1. Only add kD if Step 4 produces overshoot that can't be resolved by reducing kP
2. Start at kD=0.05
3. Increase until overshoot is eliminated
4. If the mechanism buzzes or vibrates → kD is too high, reduce it

> For most horizontal turrets, kD=0 works fine.

### Step 6: Motion Profile Limits

1. Start with the current values: 1000°/s, 7200°/s²
2. After FF and PID are tuned, gradually increase max velocity toward ~1200°/s
3. Watch for the mechanism falling behind the profile (position lags reference)
4. If it lags → your kV or motor voltage is limiting; back off velocity
5. Increase acceleration until you hear slamming or see overshoot at the endpoints, then back off 15–25%

> **Hard limit:** NEO at 5676 RPM through 27:1 = ~1261°/s theoretical max. Stay at 80–90% (1000–1130°/s) for margin.

### Verification Checklist

- [ ] Command 0°→180°→0° — position tracks setpoint with <2° error during cruise
- [ ] Command rapid 30° steps — settles within 200ms with no overshoot >5°
- [ ] Command the turret to track a slowly moving target (simulate vision input) — smooth following, no jitter
- [ ] Verify soft limits prevent rotation past safe range
- [ ] Check stator current stays within limits during aggressive moves (plot `Shooter/Turret/current`)

**AdvantageScope signals:**

| Signal | What to look for |
|--------|-----------------|
| `Shooter/Turret/angle` | Actual turret position |
| `Shooter/Turret/setpoint` | Where the turret should be |
| `Shooter/Turret/velocity` | Actual velocity — should match profile max during cruise |
| `Shooter/Turret/voltage` | Should not peg at ±12V (saturated = motor maxed out) |

**Common issues:**

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| Turret oscillates around setpoint | kP too high, or kD needed | Reduce kP, add small kD |
| Slow to reach target | kV too low or profile too conservative | Increase kV, then increase profile velocity |
| Overshoots then settles | Profile acceleration too high or kD=0 | Reduce max acceleration, add kD |
| Doesn't move at all | Gains zeroed or soft limit blocking | Check gains, verify setpoint is within soft limits |
| Slams into hard stop | Soft limits misconfigured or profile too aggressive | Verify soft limit config in YAMS, reduce profile |

**AdvantageScope:** `Shooter/Turret/angle`, `Shooter/Turret/setpoint`

---

## 3. Hood

**Motor:** TalonFX Kraken X44 (30:1) · **Control:** YAMS Pivot + MotionMagic · **FF:** kS, kV, kA
**File:** `ShooterConstants.HoodConstants` (gains), `HoodSubsystem.java` (YAMS config)

Gains: `kP=300, kD=0, kS=0.45, kV=3.0, kA=0` · Profile: `270°/s, 270°/s²`

The hood is a **small-range position mechanism** (0–30° total travel) that sets the launch angle for shots. It uses MotionMagic trapezoidal profiling on the TalonFX. Precision is critical — a 1° error at the hood translates to a significant miss at distance. The hood angles come from the distance look-up table (LUT), so it needs to hit exact positions quickly and consistently.

### Pre-flight

- [ ] Hood moves smoothly by hand with power off (no binding)
- [ ] Position reads 0° at the physical zero (verify in Phoenix Tuner X)
- [ ] Soft limits configured (prevent over-travel)
- [ ] Open AdvantageScope with `Hood/angle` and `Hood/setpoint` on same graph
- [ ] Open Phoenix Tuner X → Hood motor → Plot tab (for Position vs ClosedLoopReference overlay)

### Step 1: kS — Static Friction

1. Set all gains to 0
2. In Phoenix Tuner X, use the Control tab to slowly ramp duty cycle from 0
3. Record the voltage where the hood **just barely** moves
4. Test in both directions — kS should be similar
5. Update kS (currently 0.45)

> **Expected:** kS ≈ 0.3–0.6V. The hood has tight gearing (30:1), so friction is moderate. If the hood "sticks" at certain positions, the mechanism may have a binding spot — fix mechanically.

### Step 2: kV — Velocity Feedforward

Since this is a CTRE motor, you get the **ClosedLoopReference** signal — the actual motion profile reference trajectory. This makes kV tuning much easier than REV motors.

1. Set kP=0, kS from Step 1, kV=1.0 (starting guess)
2. Command a full-range move (0°→30°)
3. In Phoenix Tuner X or AdvantageScope, plot **Position** and **ClosedLoopReference** together
4. Zoom into the cruise (straight) segment
5. If position slope < reference slope → increase kV
6. If position slope > reference slope → decrease kV
7. Test at multiple positions in the range (0→15, 15→30, 30→0)

> **Verification:** `Hood/FX/PositionDegrees` and `Hood/FX/ReferenceDegrees` should overlap during cruise. A consistent offset means kS is slightly off.

### Step 3: kP — Proportional Gain

The hood needs high kP because:
- Short travel range (30°) means small errors matter more
- LUT angles must be hit precisely for consistent shots
- TalonFX firmware runs the PID at ~1 kHz, so high gains are stable

1. Start at kP=50
2. Command 0°→15°→0° repeatedly
3. Double kP: 50 → 100 → 200 → 400
4. Target: settle to within ±0.5° in <100ms
5. If it chatters or buzzes at the setpoint → too high, back off

> **Expected:** kP=300 is appropriate for this mechanism. CTRE Voltage mode with 30:1 gearing is very stiff. If switching to TorqueFOC, gains will be completely different.

### Step 4: kD — Derivative Gain

1. Only add kD if Step 3 produces overshoot
2. Start at kD=0.1
3. Be conservative — the hood has low inertia, so kD amplifies sensor noise easily
4. If you hear audible chatter or high-frequency vibration → kD is too high

> **Expected:** kD=0 works for most hoods. The short travel and high gearing naturally dampen oscillation.

### Step 5: kA — Acceleration Feedforward (optional)

1. If the position lags the reference during the acceleration/deceleration curves, add kA
2. Start at kA=0.01
3. The hood's short travel means there's very little time in acceleration — benefit is small
4. Only worth tuning if you're pushing the profile limits higher

> **Expected:** kA=0 is fine for most setups. The 30° range means the hood spends most of its move in acceleration/deceleration with minimal cruise.

### Step 6: Motion Profile Limits

1. Start with 270°/s, 270°/s² (current values)
2. After tuning, increase velocity if shot transitions feel slow (e.g., changing distance during tracking)
3. The hood covers 0–30°, so at 270°/s the full range takes ~0.11s — already fast
4. Increase acceleration for crisper starts/stops
5. Watch for position overshooting the setpoint → acceleration too high

> **Theoretical max:** Kraken X44 at 6000 RPM / 30:1 ≈ 1200°/s, but the hood doesn't need anywhere near that.

### Verification Checklist

- [ ] Command each LUT distance angle — hood hits within ±0.5° every time
- [ ] Rapid angle changes (simulating distance changes during tracking) — settles in <100ms
- [ ] No audible chatter or vibration at any setpoint
- [ ] Stator current stays within limits (plot in AdvantageScope)
- [ ] Position graph shows clean rectangular shape for step commands

**AdvantageScope signals:**

| Signal | What to look for |
|--------|-----------------|
| `Hood/angle` | Actual hood position |
| `Hood/setpoint` | Target position from LUT |
| `Hood/FX/PositionDegrees` | Raw TalonFX position (matches `angle`) |
| `Hood/FX/ReferenceDegrees` | MotionMagic reference — the kV/kA tuning line |
| `Hood/FX/StatorCurrent` | Should not saturate during normal moves |
| `Hood/FX/Voltage` | Should not peg at ±12V |

**Common issues:**

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| Hood chatters at setpoint | kP too high or kD amplifying noise | Reduce kP, remove kD |
| Slow to reach angle | kV too low or profile too slow | Increase kV, increase profile limits |
| Shots miss at specific distances | Hood not reaching exact LUT angle | Tighten tolerance, check encoder calibration |
| Hood drifts after reaching setpoint | Mechanical backlash or FF imbalance | Add small kI (last resort), check mechanism |

---

## 4. Flywheel

**Motors:** 2× TalonFX (leader + inverted follower) · **Control:** YAMS FlyWheel (velocity) · **FF:** kS, kV, kA
**File:** `ShooterConstants.FlywheelConstants` (gains), `FlywheelSubsystem.java` (YAMS config)

Gains: `kP=0.2, kD=0, kS=0.35, kV=0.12, kA=0`

The flywheel is a **velocity-controlled** mechanism — no motion profile, no position target. FF terms multiply the velocity setpoint directly (unlike position mechanisms where they multiply the profile's planned velocity). Shot consistency depends on the flywheel being at the exact RPM when the ball is released.

### Pre-flight

- [ ] Flywheel spins freely by hand (no rubbing, bearings smooth)
- [ ] Follower motor is inverted correctly (both spin the same direction)
- [ ] Current limits set (prevent brownouts — flywheel draws the most current on the robot)
- [ ] Open AdvantageScope with `Flywheel/velocity` and `Flywheel/setpoint` on same graph

### Step 1: kS — Static Friction

1. Set all gains to 0
2. **On the real robot** (kS can't be found in sim — there's no friction):
   - In Phoenix Tuner X, slowly ramp duty cycle from 0
   - Record the voltage where the flywheel **just barely** starts spinning
3. Test from a dead stop multiple times and average
4. Update kS

> **Expected:** kS ≈ 0.2–0.5V for dual Kraken X60.

### Step 2: kV — Velocity Feedforward

This is the most important gain for velocity mechanisms. kV converts RPM → voltage.

1. Set kP=0, kS from Step 1, kV=0.05 (starting guess)
2. Command a moderate RPM (e.g., 2000 RPM) via the tunable `Shooter/FlywheelRPM`
3. Wait for steady state (5+ seconds) — measure actual RPM
4. **Formula:** `kV = (voltage_applied - kS) / actual_RPM_in_native_units`
5. Verify at **multiple speeds** — kV should work across the range:

| Test RPM | Expected behavior |
|----------|------------------|
| 1000 | Steady at ±30 RPM |
| 2000 | Steady at ±30 RPM |
| 3000 | Steady at ±30 RPM |
| 4000 | Steady at ±30 RPM (if battery is fresh) |

6. If steady-state error is consistent across all speeds → kV is off
7. If error is only at low speeds → kS is off
8. If error grows at high speeds → motor saturation, reduce max RPM target

> **Verification:** Plot `Flywheel/FX/VelocityRPM` vs `Flywheel/FX/ReferenceRPM` in AdvantageScope. With kP=0, the gap is purely kV/kS error.

### Step 3: kP — Proportional Gain

kP handles transient errors: spin-up time and recovery after a ball passes through.

1. Set kP=0.05
2. Command 0→3000 RPM (cold start spin-up)
3. Measure time to reach setpoint (within tolerance)
4. Double kP: 0.05 → 0.1 → 0.2 → 0.4
5. Target: reach setpoint in **<0.5s** with no velocity oscillation
6. Test **recovery**: while spinning at 3000 RPM, fire a ball — RPM should dip and recover in <0.3s

> **Expected:** kP=0.2 is conservative. Reference teams use 0.5–1.0. Try increasing to 0.5 if spin-up is slow.

> **Oscillation check:** If velocity graph wobbles around the setpoint (±50 RPM saw-tooth), kP is too high. Flywheels have high inertia — they don't stop oscillating easily once they start.

### Step 4: kA — Acceleration Feedforward (optional)

kA helps with spin-up by providing extra voltage proportional to the desired acceleration.

1. Only needed if spin-up time is critical and kP alone isn't fast enough
2. Start at kA=0.001
3. Increase until spin-up time improves without overshoot
4. kA fights kP at steady state — too much kA causes RPM to overshoot then settle

> **Expected:** kA=0 is fine for most setups. The flywheel's high inertia smooths everything.

### Step 5: Tolerance Tuning

The flywheel "ready" signal (`Flywheel/IsSpunUp`) gates when the robot can shoot. Too tight → takes too long to report ready. Too loose → shots are inconsistent.

1. Current tolerance: 30 RPM
2. Fire multiple shots at the same distance with 30 RPM tolerance
3. If shots are inconsistent → tighten to 20 RPM or 15 RPM
4. If the robot hesitates before shooting (waiting for flywheel) → loosen to 40–50 RPM
5. Balance: tightest tolerance that doesn't slow down shot cadence

### Verification Checklist

- [ ] kV verified at 4 different RPMs (1000, 2000, 3000, 4000)
- [ ] Spin-up 0→3000 RPM in <0.5s
- [ ] Recovery after shot: dip <200 RPM, recover in <0.3s
- [ ] No velocity oscillation at steady state
- [ ] `IsSpunUp` goes true reliably before each shot
- [ ] Battery at 12V vs 11.5V doesn't change steady-state accuracy (FF should compensate)

**AdvantageScope signals:**

| Signal | What to look for |
|--------|-----------------|
| `Flywheel/velocity` | Actual flywheel speed |
| `Flywheel/setpoint` | Commanded speed |
| `Flywheel/FX/VelocityRPM` | Raw TalonFX velocity |
| `Flywheel/FX/ReferenceRPM` | CTRE reference — gap from actual = FF error |
| `Flywheel/FX/StatorCurrent` | Current draw — watch for brownout risk |
| `Flywheel/IsSpunUp` | Boolean — should be true before each shot |

**Common issues:**

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| Slow spin-up | kP too low | Increase kP to 0.5+ |
| RPM oscillates at steady state | kP too high | Reduce kP, verify kV is accurate |
| Steady-state error at all speeds | kV wrong | Recharacterize at multiple RPMs |
| Error only at low RPM | kS wrong | Recharacterize kS from dead stop |
| RPM drops too much on shot | kP too low or kA=0 | Increase kP, consider adding kA |
| Shots inconsistent | Tolerance too loose or kV inaccurate | Tighten tolerance, verify kV |

**Tunable:** `Shooter/FlywheelRPM`

---

## 5. Kicker

**Motor:** SparkFlex + NEO Vortex · **Control:** YAMS FlyWheel (FF-only, PID=0) · **FF:** kS, kV
**File:** `ShooterConstants.KickerConstants` (gains), `KickerSubsystem.java` (YAMS config)

Gains: `kP=0, kS=0.25, kV=0.12`

The kicker feeds balls from the spindexer into the flywheel. It's an **FF-only velocity mechanism** — kP=0 means the kicker runs open-loop with just feedforward. This is intentional: the kicker only needs consistent speed, not precise velocity tracking.

### Why FF-only?

The kicker's job is simple: spin at a constant speed to push the ball into the flywheel. Small velocity variations don't affect shot quality — the flywheel dominates. Adding PID adds complexity and potential oscillation for no benefit.

### Tuning kS and kV

1. Set kS=0, kV=0
2. Use YAMS Live Tuning to slowly increase voltage until the kicker **just starts spinning**
3. That voltage is kS (~0.25V)
4. Set a target RPM via `Kicker/KickerRPM` tunable
5. With only kS and kV active (kP=0), adjust kV until actual RPM is within ~50 RPM of target at steady state
6. Verify at 2-3 different RPM targets

> **Expected:** kS ≈ 0.2–0.4V, kV ≈ 0.10–0.15 for NEO Vortex.

### When to add kP

Add kP only if you observe:
- Kicker **stalls or slows significantly** when a ball contacts it (ball loading causes a torque spike)
- Feeding becomes inconsistent — some balls enter the flywheel too slowly

If needed:
1. Start kP=0.01
2. Run the kicker and feed balls repeatedly
3. Increase until RPM dip on ball contact is <10% of setpoint
4. Don't go higher than needed — the kicker doesn't need to be precise

### Verification

- [ ] Kicker runs at consistent speed (±50 RPM at steady state with FF-only)
- [ ] Ball feeds cleanly into flywheel without hesitation
- [ ] No stall or significant slowdown on ball contact
- [ ] RPM is sufficient to push the ball through (increase target if balls jam)

**AdvantageScope:** `Kicker/velocity`, `Kicker/setpoint` · **Tunable:** `Kicker/KickerRPM`

**Common issues:**

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| Ball doesn't enter flywheel | Kicker RPM too low | Increase target RPM |
| Kicker stalls on ball contact | Insufficient torque — need kP | Add kP=0.01, increase if needed |
| Kicker oscillates | kP too high (shouldn't happen with kP=0) | Remove kP, verify FF only |

---

## 6. Spindexer

**Motor:** SparkMax + NEO (5:1) · **Control:** YAMS FlyWheel (velocity) · **FF:** kS, kV
**File:** `ShooterConstants.SpindexerConstants` (gains), `SpindexerSubsystem.java` (YAMS config)

Gains: `kP=0.02, kS=0.25, kV=0.6`

The spindexer rotates balls inside the magazine to feed them to the kicker. It's a low-speed velocity mechanism with **COAST idle mode** — when not commanded, the spindexer freewheels. This is intentional: it prevents balls from being held under compression.

### Why COAST mode?

BRAKE mode would hold the spindexer in place when idle, compressing balls between the spindexer walls. This can:
- Deform soft game pieces over time
- Cause jams when restarting
- Increase motor current for no benefit

COAST lets the spindexer spin freely, keeping balls loose and ready to feed.

### Tuning kS and kV

Follow the same process as other velocity mechanisms:

1. Set kS=0, kV=0, kP=0
2. Slowly increase voltage until the spindexer **just starts moving** → that's kS (~0.25V)
3. Set a target RPM via `Spindexer/IndexerRPM` tunable
4. Adjust kV until actual RPM matches at steady state (~0.6)
5. Verify at 2–3 RPM targets to confirm kV is linear

> **Expected:** kV ≈ 0.4–0.8 for NEO through 5:1. The low gear ratio means relatively high kV.

### Tuning kP

The current kP=0.02 is very low — nearly FF-only. This is fine if feeding is consistent, but increase if:
- Balls feed at inconsistent rates (some fast, some slow)
- The spindexer stalls when multiple balls load simultaneously
- RPM drops >20% when a ball engages the kicker

**To increase:**
1. Start at kP=0.05
2. Feed balls and watch RPM in AdvantageScope
3. Increase until RPM dip on ball contact is <10%
4. Don't go too high — the spindexer doesn't need tight velocity tracking

### Verification

- [ ] Spindexer runs at consistent RPM (±30 RPM at steady state)
- [ ] Balls feed smoothly to the kicker — no jams or hesitation
- [ ] COAST mode works — spindexer freewheels when not commanded
- [ ] No stall when multiple balls load simultaneously
- [ ] Feeding rate is consistent (balls arrive at kicker at regular intervals)

**AdvantageScope:** `Spindexer/velocity`, `Spindexer/setpoint` · **Tunable:** `Spindexer/IndexerRPM`

**Common issues:**

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| Balls jam in magazine | Spindexer RPM too low or mechanical interference | Increase target RPM, check for physical obstructions |
| Inconsistent feed rate | kP too low — RPM varies with load | Increase kP to 0.05–0.1 |
| Spindexer doesn't start | kS too low | Increase kS, check wiring |
| Balls deform over time | BRAKE mode accidentally set | Verify COAST idle mode in YAMS config |

---

## 7. Intake Rollers

**Motor:** TalonFX Kraken X60 (2:1) · **Control:** YAMS FlyWheel (velocity) · **FF:** kS, kV
**File:** `IntakeConstants.Rollers` (gains), `IntakeRollerSubsystem.java` (YAMS config)

Gains: `kP=0.5, kS=0.39, kV=0.24`

The intake rollers pull balls from the ground into the robot. They're a velocity mechanism with relatively aggressive kP (0.5) because the rollers experience large, sudden load changes when a ball contacts them. The 2:1 gear ratio provides high speed with moderate torque.

### Tuning kS and kV

1. Set kS=0, kV=0, kP=0
2. In Phoenix Tuner X, ramp duty cycle until rollers barely spin → kS (~0.39V)
3. Set a target RPM and adjust kV until steady-state matches (~0.24)
4. Verify at 2–3 RPM targets

> **Expected:** kS ≈ 0.3–0.5V, kV ≈ 0.2–0.3 for Kraken X60 with 2:1 ratio.

### Tuning kP

The rollers need higher kP than other velocity mechanisms because:
- Ball contact causes sudden, large torque spikes
- If the rollers slow too much, the ball doesn't enter reliably
- The 2:1 ratio provides less mechanical advantage — the motor feels the load more directly

1. Start at kP=0.1
2. Run rollers at target RPM and repeatedly feed balls
3. Watch RPM dip in AdvantageScope when each ball contacts
4. Increase kP until dip is <15%: 0.1 → 0.2 → 0.5 → 1.0
5. If rollers buzz or oscillate at steady state → kP too high

> **Expected:** kP=0.5 is a good starting point. Reduce only if you observe oscillation.

### Verification

- [ ] Rollers spin at target RPM (±30 RPM at steady state)
- [ ] Ball pickup is reliable — balls enter consistently from the ground
- [ ] RPM dip on ball contact is <15% and recovers in <0.2s
- [ ] No oscillation at steady state
- [ ] Stator current stays within limits during ball contact (plot `IntakeRollers/current`)

**AdvantageScope signals:**

| Signal | What to look for |
|--------|-----------------|
| `IntakeRollers/velocity` | Actual roller speed |
| `IntakeRollers/setpoint` | Commanded speed |
| `IntakeRollers/FX/StatorCurrent` | Current spikes on ball contact — should not trip limits |

**Common issues:**

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| Ball doesn't enter robot | RPM too low or rollers stall on contact | Increase target RPM, increase kP |
| Rollers oscillate | kP too high | Reduce kP to 0.3 |
| Ball enters but jams further in | Spindexer not running or too slow | Check spindexer, not a roller issue |
| High current draw at idle | Mechanical binding or debris | Inspect rollers physically |

---

## 8. Intake Pivot

**Motors:** 2× SparkFlex + NEO Vortex (30:1) · **Control:** YAMS Arm + ExponentialProfile · **FF:** ArmFeedforward(kS, kG, kV) — gravity-compensated
**File:** `IntakeConstants.Pivot` (gains), `IntakePivotSubsystem.java` (YAMS config)

Gains: `kP=0, kG=0.21, kS=0.11, kV=0, kA=0` · Profile: `90°/s, 90°/s²`

> ⚠️ **PID is all zeros and FF is commented out.** Priority: uncomment `.withFeedforward(armFeedforward())` and `.withSimFeedforward(armFeedforward())` in `IntakePivotSubsystem.java` (~line 123–124), tune kG, then add PID.

The intake pivot is the most complex mechanism to tune because it combines **gravity compensation** (kG), an **exponential motion profile**, and **high inertia** (two motors, 30:1 gearing, a heavy arm). It's configured as a YAMS `Arm` type, which means YAMS automatically applies `kG × cos(angle)` — gravity compensation that varies with position.

### Understanding the Exponential Profile on Spark

The intake pivot uses an **exponential profile** running on the roboRIO (not in the Spark firmware). YAMS runs this in a dedicated `Notifier` thread — each tick it:
1. Calculates the next `ExponentialProfile` state (position + velocity)
2. Runs PID on the position error
3. Adds feedforward (kG×cos(angle) + kS×sign(velocity) + kV×velocity)
4. Sends the total as a voltage command to the SparkFlex

You'll see `"====== Spark(X) Using RIO Closed Loop Controller ======"` in the console at startup. This is normal — it means the exponential profile is active and the Spark's onboard PID is bypassed.

The profile parameters (90°/s, 90°/s²) are **starting points**. Unlike trapezoidal profiles where you set max velocity and max acceleration directly, exponential profiles derive their shape from motor physics. The 90°/s and 90°/s² values are used with `withExponentialProfile(maxVoltage, maxVelocity, maxAccelAtStall)`.

### Pre-flight

- [ ] Pivot moves freely by hand with power off (no binding at any angle)
- [ ] Absolute encoder reads the correct angle at stow position
- [ ] Soft limits set (prevent pivot from going past stow or past full deploy)
- [ ] **Feedforward is uncommented** in `IntakePivotSubsystem.java`:
  ```java
  .withFeedforward(armFeedforward())
  .withSimFeedforward(armFeedforward())
  ```
- [ ] `tuningMode = true` in `Constants.java`
- [ ] Open AdvantageScope with `IntakePivot/angle` and `IntakePivot/setpoint` on the same graph
- [ ] Have someone ready on the E-stop — a heavy arm with bad gains can be dangerous

### Step 1: kG — Gravity Compensation (MOST IMPORTANT)

kG must be tuned **first and precisely**. For arms, YAMS applies `kG × cos(angle)`, which varies the gravity compensation based on the arm's angle. At horizontal (0°), you need full compensation. At vertical (90°), you need none.

1. Set ALL gains to 0 (kP, kD, kS, kV, kA, kG)
2. **Manually hold the arm horizontal** (or at a known angle)
3. Set kG=0.05 via YAMS Live Tuning
4. Slowly release the arm — does it drop, hold, or rise?
   - **Drops:** kG too low → increase
   - **Rises:** kG too high → decrease
   - **Holds:** kG is close
5. Binary search: double if too low, halve if too high
6. Narrow to **2–3 decimal places** (e.g., 0.21)
7. Test at **multiple angles** — YAMS applies cos(angle) automatically, so if kG is correct at horizontal, it should be correct everywhere

> **Expected:** kG ≈ 0.15–0.30V for two NEO Vortex through 30:1. Our value of 0.21 is plausible but should be verified on the real robot.

> **⚠️ Precision matters.** A 10% error in kG means the arm constantly drifts, and PID has to fight gravity instead of just correcting position error. Spend extra time here.

### Step 2: kS — Static Friction

1. With kG set, the arm should hold position at any angle
2. Slowly increase voltage (via YAMS Live Tuning) from kG until the arm **just barely** starts moving
3. `kS = that voltage - kG × cos(current_angle)`
4. Test in both directions (up and down) and average
5. Currently kS=0.11 — verify

> **Expected:** kS ≈ 0.05–0.20V. The 30:1 gearing adds significant friction.

### Step 3: kV — Velocity Feedforward

1. Set kP=0 (FF-only)
2. Command a slow, constant-velocity move (deploy → stow)
3. `kV = (voltage - kG×cos(θ) - kS) / velocity`
4. Currently kV=0 — this means the profile has no velocity feedforward
5. Start at kV=0.5, command deploy → stow, and check if the arm moves at the profile's planned velocity
6. Increase until the arm's velocity during the cruise phase matches the profile

> **Currently kV=0** — the arm relies entirely on kP to track the profile. Adding kV will significantly improve tracking and allow lower kP.

### Step 4: kP — Proportional Gain

1. Set kP=0.5 (start low — this is a heavy arm)
2. Command stow → deploy (full range of motion)
3. Watch the position graph — should follow the setpoint profile
4. Double kP: 0.5 → 1.0 → 2.0 → 4.0
5. Target: settle time <0.3s with no overshoot
6. The exponential profile smooths the trajectory, so kP shouldn't need to be very high

> **Expected:** kP ≈ 1.0–5.0. Currently kP=0, which means the arm has **no position correction at all** — only FF. This must be fixed.

### Step 5: kD — Derivative Gain

The intake pivot has **significant inertia** — expect to need kD.

1. If Step 4 produces overshoot, add kD
2. Start at kD=0.05
3. Increase until overshoot is eliminated: 0.05 → 0.1 → 0.2
4. Heavy arms benefit from kD more than light mechanisms
5. If the arm vibrates or buzzes → kD too high

> **Expected:** kD ≈ 0.05–0.2. Unlike the turret or hood, this mechanism has enough inertia to need active damping.

### Step 6: Motion Profile Limits

1. Current values: 90°/s, 90°/s² — very conservative
2. After kG, kV, kP, and kD are tuned and the arm holds reliably:
3. Increase max velocity: 90 → 150 → 200°/s
4. Increase max acceleration: 90 → 150 → 200°/s²
5. Watch for the arm falling behind the profile or overshooting endpoints
6. The exponential profile naturally limits acceleration based on motor physics, so you can be more aggressive than with trapezoidal

> **Theoretical max:** 2× Vortex at 6784 RPM / 30:1 ≈ 1358°/s output. Practical limit depends on arm weight and desired smoothness. 200–300°/s is a reasonable target for a heavy arm.

### Switching to Motor-Physics Exponential Profile (future)

The current configuration uses `withExponentialProfile(maxVoltage, maxVelocity, maxAccelAtStall)` with manually specified constraints. For a more physics-accurate profile, switch to:

```java
.withExponentialProfile(Volts.of(12), DCMotor.getNeoVortex(2), kMomentOfInertia)
```

This derives the constraints from the motor's actual torque-speed curve and the mechanism's moment of inertia. Benefits:
- Profile automatically adjusts for motor saturation
- More accurate at high speeds
- Better gravity-aware behavior (accelerates faster going down)

To use this, you need to measure or calculate the arm's moment of inertia (use CAD or swing test).

### Verification Checklist

- [ ] Arm holds position at any angle with kG only (kP=0) — no drifting
- [ ] kG correct at horizontal AND at 45° (cos scaling works)
- [ ] Deploy → stow in <0.5s with no overshoot
- [ ] Stow → deploy in <0.5s with no slam at the end
- [ ] Arm doesn't drift when idle (kG compensating gravity)
- [ ] Profile tracking is smooth — no jerkiness during the move
- [ ] Stator current stays within limits (plot in AdvantageScope)
- [ ] Test with a ball in the intake — added weight shouldn't change behavior significantly

**AdvantageScope signals:**

| Signal | What to look for |
|--------|-----------------|
| `IntakePivot/angle` | Actual pivot position |
| `IntakePivot/setpoint` | Target position |
| `IntakePivot/velocity` | Actual velocity — compare to profile max during cruise |
| `IntakePivot/voltage` | Total voltage output — verify not saturating at ±12V |

**Common issues:**

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| Arm falls when released | kG too low or FF commented out | Increase kG, verify `.withFeedforward()` is uncommented |
| Arm drifts up slowly | kG too high | Decrease kG |
| Arm oscillates at setpoint | kP too high, kD too low | Reduce kP, add kD |
| Arm slams at end of travel | Profile limits too aggressive or kD=0 | Reduce acceleration, add kD, check soft limits |
| Arm moves but doesn't track profile | kV=0 (current state!) | Add kV — arm relies only on kP without it |
| Arm holds but won't move | kP=0 (current state!) | Add kP — FF alone can't correct position errors |
| "Using RIO Closed Loop Controller" message | Normal — exponential profile running on RIO | Not a problem — expected behavior |

---

## 9. PathPlanner

See [Step 8: PathPlanner Configuration](#step-8-pathplanner-configuration) in the Drivetrain section above. PathPlanner tuning depends entirely on having accurate drivetrain gains first.

---

## Pigeon IMU Calibration

The Pigeon 2 IMU tells the code which direction the robot is facing. If misconfigured, **field-relative driving, autonomous, and vision pose estimation will all be wrong.**

### MountPose

The Pigeon expects flat mounting with its arrow forward. If mounted differently, configure `MountPose`:

Reference:
- **LASA PR:** roll=179.96°, pitch=1.14°, yaw=-90.45° (upside-down, rotated 90°)
- **WHS 3467:** roll=180° (upside-down, arrow forward)
- **Hammerheads:** yaw=-90° (flat, arrow pointing left)

**To configure:**

1. Inspect the Pigeon: which way does the arrow point? Is it flat or flipped?
2. In Phoenix Tuner X → Pigeon 2 → Configs → MountPose:
   - **Yaw:** 0° = arrow forward, -90° = arrow left, 90° = arrow right
   - **Roll:** 0° = right-side-up, 180° = upside-down
   - **Pitch:** Usually 0°
3. Apply and save

Or configure in code:
```java
var pigeonConfig = new Pigeon2Configuration();
pigeonConfig.MountPose.MountPoseYaw = -90;  // arrow points left
pigeonConfig.MountPose.MountPoseRoll = 180; // upside-down
pigeonConfig.MountPose.MountPosePitch = 0;
```

### Startup & Verification

- Let the robot sit 2–3 minutes after power-on (auto-calibrates on startup)
- Don't bump/move during first few seconds
- Zero yaw before match (driver reset button calls `pigeon.setYaw(0)`)
- **Verify:** Face a known direction, rotate 90° left → should read +90°, rotate right → -90°

> ⚠️ If mounted flat with arrow forward, MountPose is not needed.

---

## PhotonVision Camera Tuning

Camera settings affect AprilTag detection range and reliability under varying lighting.

### Resolution vs. FPS

| Resolution | Range | FPS | Use |
|:----------:|:-----:|:---:|-----|
| 640×480 | ~3m | 60+ | Close range |
| **1280×960** | **~5m** | **20–30** | **Recommended** |
| 1600×1200 | ~7m | 10–15 | Max range |
| 1800×1200 | ~7m+ | 8–12 | 6328 mono cameras |

### Key Settings (`http://photonvision.local:5800`)

| Setting | Recommendation | Notes |
|---------|---------------|-------|
| **Exposure** | 3000–5000µs | Lower = sharper but darker. Reduce if tags blur at speed. |
| **Gain** | 5 (color), 10–15 (mono) | Higher = brighter but noisier |
| **LEDs** | OFF | Glare washes out tags |
| **Target model** | AprilTag 36h11 | FRC standard |

**6328 reference:** Color 1280×960 / exposure=4500µs / gain=5. Mono 1800×1200 / exposure=1800µs / gain=15.

### Multi-Camera

Each camera adds CPU load — monitor in web interface. Stagger resolutions if needed. Our code already handles multi-camera fusion.

### Verification

1. Place AprilTag at known distance (3m) → verify detection in web interface
2. Check estimated pose in AdvantageScope (should be within ~5cm at 3m)
3. Test rapid rotation — if tags blur, reduce exposure

> **📋 Arena lighting varies wildly.** Bring a laptop and adjust exposure/gain during practice matches.

---

## Elastic Dashboard for Matches

[Elastic](https://github.com/Gold872/elastic-dashboard) replaces Shuffleboard — faster and works well with NT4. Use for **in-match awareness**, not tuning (use AdvantageScope for that).

### How Our NT Keys Work

Our code publishes telemetry via AdvantageKit:

| Method | NT Prefix | Used By |
|--------|-----------|---------|
| `Logger.recordOutput("Key", val)` | `/AdvantageKit/Key` | AdvantageScope replay, Elastic widgets |
| `SmartDashboard.putX("Key", val)` | `/SmartDashboard/Key` | Field2d, Auto Chooser, DriveIsMoving (template code) |

**Important:** Elastic can subscribe to **any** NT4 topic, including `/AdvantageKit/...` — it does NOT require `/SmartDashboard/`. Our `elastic-layout.json` points directly at `/AdvantageKit/` paths for all data published via `Logger.recordOutput`. Only the Field2d widget, Auto Chooser, and DriveIsMoving (published by the drive template) use `/SmartDashboard/`.

In Elastic, most widget NT paths start with `/AdvantageKit/`. In AdvantageScope, the same paths appear under `AdvantageKit/`.

### Recommended Match Layout (6328-Style)

6328 (Mechanical Advantage) publishes hub shift data to their dashboard for operator awareness during 2026 Rebuilt's alternating shift windows. Our code does the same via `HubShiftUtil`. We ship a ready-to-use layout in `src/main/deploy/elastic-layout.json`.

#### Row 1 — Match Awareness (biggest widgets, top of screen)

| Widget Type | NT Key (Elastic) | Elastic Widget | Notes |
|-------------|------------------|----------------|-------|
| Match Timer | `/AdvantageKit/MatchTime` | **Number** (large font) | Countdown from FMS. Most important number. |
| Shift Timer | `/AdvantageKit/HubShift/ShiftedRemainingTime` | **Number** (large font) | Seconds until current shift ends (TOF-adjusted). |
| Shift Active | `/AdvantageKit/HubShift/ShiftedActive` | **Boolean Box** | Green = our alliance can score. Red = opponent's turn. |
| Game State | `/AdvantageKit/HubShift/CurrentShift` | **Text Display** | TRANSITION / SHIFT1 / SHIFT2 / SHIFT3 / SHIFT4 / ENDGAME |
| Hub Active | `/AdvantageKit/HubShift/Active` | **Boolean Box** | Green when official shift is active. |

#### Row 2 — Shooter Status

| Widget Type | NT Key (Elastic) | Elastic Widget | Notes |
|-------------|------------------|----------------|-------|
| Flywheel Ready | `/AdvantageKit/Flywheel/IsSpunUp` | **Boolean Box** | Green = RPM on target. |
| Flywheel RPM | `/AdvantageKit/Flywheel/FX/VelocityRPM` | **Number** | Current flywheel speed. |
| Hood Angle | `/AdvantageKit/Hood/FX/PositionDegrees` | **Number** | Current hood position in degrees. |
| Turret Angle | `/AdvantageKit/Shooter/Turret/PositionDegrees` | **Number** | Current turret position in degrees. |

#### Row 3 — Mechanisms & System Health

| Widget Type | NT Key (Elastic) | Elastic Widget | Notes |
|-------------|------------------|----------------|-------|
| Battery | `/AdvantageKit/Battery/Voltage` | **Number** | Flash red < 11.5V. |
| Drive Moving | `/SmartDashboard/Mechanisms/DriveIsMoving` | **Boolean Box** | Green = actively driving. (Drive template uses SmartDashboard.) |
| All Mechanisms | `/AdvantageKit/Mechanisms/*IsMoving` | **Boolean Box** (each) | Quick at-a-glance mechanism health. |
| Field View | `/SmartDashboard/Field` | **Field Widget** | 2D robot pose — verify auto worked. (Field2d uses SmartDashboard.) |

### Elastic Setup Step-by-Step

1. **Install:** Elastic is included with the FRC Game Tools installer — no separate download needed. If you need the latest version, grab it from [GitHub releases](https://github.com/Gold872/elastic-dashboard/releases).
2. **Connect:** Launch Elastic → it auto-discovers the robot via NT4 (same as Shuffleboard). In sim, it connects to `localhost`.
3. **Load our layout:** File → Open Layout → select `src/main/deploy/elastic-layout.json` from the repo. This ships with three pre-built tabs:
   - **Teleop** — hub shift timers, shooter status, mechanism health, field view
   - **Auto** — auto chooser, match time, field view with trajectory
   - **Disabled** — auto chooser, field view
4. **Customise widgets:**
   - Right-click a widget → **Properties** → change colors or font sizes.
   - For Boolean Box: set true-color = green, false-color = red.
   - For Number widgets: increase font size to **48–72pt** (driver is 10+ feet away).
5. **Save layout:** File → Save Layout → commit changes to the `.json` file so the whole team uses the same layout.
6. **Lock before competition:** Right-click → Lock All Widgets. Prevents accidental drag during matches.

### Auto Tab Switching

Our `Robot.java` already calls `Elastic.selectTab()` on mode transitions:

```
disabledInit()  → Elastic.selectTab("Disabled")
autonomousInit() → Elastic.selectTab("Auto")
teleopInit()    → Elastic.selectTab("Teleop")
```

Name your Elastic tabs exactly **"Disabled"**, **"Auto"**, and **"Teleop"** (case-sensitive) to match.

### Design Principles

1. **Less is more.** Only show what the driver/operator will actually look at mid-match.
2. **Use color.** Green = good, Red = bad. Booleans > numbers for at-a-glance status.
3. **Big text.** 48pt minimum for numbers the driver needs. They're 10+ feet away and stressed.
4. **Test it.** Have the driver stand at normal distance — if they can't read it, make it bigger.
5. **Separate tuning from matches.** Never clutter the match layout with gains or debug data. Use AdvantageScope for that.

> 💡 **Debug tab:** Optionally add a fourth tab with detailed data (motor temps, CAN utilization, per-module velocities). Hide it during competition.

---

## Quick Reference Table

| Mechanism | Motor(s) | Control | FF Terms | PID | Motion Profile | Profile Type |
|-----------|----------|---------|----------|-----|---------------|-------------|
| Drive (steer) | TalonFX | Position | kS, kV | kP, kD | MotionMagicExpo | Exponential (firmware) |
| Drive (drive) | TalonFX | Velocity | kS, kV | kP | — | — |
| Turret | NEO (27:1) | Position+MP | kS, kV, kA | kP | 1000°/s, 7200°/s² | Trapezoidal (firmware) |
| Hood | TalonFX (30:1) | Position+MP | kS, kV | kP | 270°/s, 270°/s² | Trapezoidal (firmware) |
| Flywheel | 2× TalonFX | Velocity | kS, kV | kP | — | — |
| Kicker | Vortex | Velocity (FF-only) | kS, kV | — | — | — |
| Spindexer | NEO (5:1) | Velocity | kS, kV | kP | — | — |
| Intake Rollers | Kraken X60 (2:1) | Velocity | kS, kV | kP | — | — |
| Intake Pivot | 2× Vortex (30:1) | Position+MP | kG, kS, kV | ⚠️ kP=0 | 90°/s, 90°/s² | Exponential (RIO) |

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
