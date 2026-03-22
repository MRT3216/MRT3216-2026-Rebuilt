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

Gains: `kP=3.0, kD=0, kS=0, kV=1.0, kA=0.05` · Profile: `1000°/s, 7200°/s²`

> ⚠️ Profile reduced from 1440°/s — NEO through 27:1 maxes out at ~1261°/s.

1. **kS:** Horizontal — no gravity. Find minimum voltage to start moving. (0)
2. **kV:** Match Position vs Reference slopes during cruise. (1.0)
3. **kA:** Match curved accel portions. Start 0.001. (0.05)
4. **kP:** Command 0°→90°, settle <200ms. (3.0)
5. **kD:** Add if overshoot. Start 0.05.
6. **Profile:** 1000°/s ≈ 0.36s for 360°. Can increase to ~1200°/s.

**AdvantageScope:** `Shooter/Turret/angle`, `Shooter/Turret/setpoint`

---

## 3. Hood

**Motor:** TalonFX Kraken X44 (30:1) · **Control:** YAMS Pivot + MotionMagic · **FF:** kS, kV, kA

Gains: `kP=300, kD=0, kS=0.45, kV=3.0, kA=0` · Profile: `270°/s, 270°/s²`

1. **kS:** Already 0.45. Verify with slow sweep — smooth from standstill.
2. **kV:** Match Position vs Reference in Tuner X. kV is actively used by MotionMagic. (3.0)
3. **kP:** High (300) for precise LUT angles. Command 0°→15°→0°, settle <100ms.
4. **kD:** Only if audible chatter. Conservative — low inertia amplifies noise.
5. **Profile:** 270°/s is moderate. Increase if sluggish during rapid LUT changes.

**AdvantageScope:** `Hood/angle`, `Hood/setpoint`

---

## 4. Flywheel

**Motors:** 2× TalonFX (leader + inverted follower) · **Control:** YAMS FlyWheel (velocity) · **FF:** kS, kV, kA

Gains: `kP=0.2, kD=0, kS=0.35, kV=0.12, kA=0`

1. **kS:** Ramp voltage until flywheel barely spins. Record it. *(Skip in sim.)*
2. **kV:** Set target velocity (kP=0), adjust until steady-state matches. Test at 1000, 2000, 3000, 4000 RPM.
3. **kP:** Step velocity command, increase until setpoint reached in ~0.5s. (0.2)
4. **kA:** Only for faster spin-up.
5. **Tolerance:** 30 RPM. Tighten if shots inconsistent (costs spin-up time).

**AdvantageScope:** `Flywheel/velocity`, `Flywheel/setpoint` · **Tunable:** `Shooter/FlywheelRPM`

---

## 5. Kicker

**Motor:** SparkFlex + NEO Vortex · **Control:** YAMS FlyWheel (FF-only, PID=0) · **FF:** kS, kV

Gains: `kP=0, kS=0.25, kV=0.12`

Just needs consistent speed to feed balls. Tune kS/kV like flywheel. Only add kP if kicker slows on ball contact. **Tunable:** `Kicker/KickerRPM`

---

## 6. Spindexer

**Motor:** SparkMax + NEO (5:1) · **Control:** YAMS FlyWheel (velocity) · **FF:** kS, kV

Gains: `kP=0.02, kS=0.25, kV=0.6`

Tune kS/kV like flywheel. kP=0.02 is very low — increase if balls feed inconsistently. COAST idle mode (freewheels when not commanded — intentional). **Tunable:** `Spindexer/IndexerRPM`

---

## 7. Intake Rollers

**Motor:** TalonFX Kraken X60 (2:1) · **Control:** YAMS FlyWheel (velocity) · **FF:** kS, kV

Gains: `kP=0.5, kS=0.39, kV=0.24`

Tune kS/kV like other velocity mechanisms. kP=0.5 is aggressive — verify by feeding balls. Slows too much on contact → increase kP. Buzzes/oscillates → decrease.

---

## 8. Intake Pivot

**Motors:** 2× SparkFlex + NEO Vortex (30:1) · **Control:** YAMS Arm + ExponentialProfile · **FF:** ArmFeedforward(kS, kG, kV) — gravity-compensated

Gains: `kP=0, kG=0.21, kS=0.11, kV=0, kA=0` · Profile: `90°/s, 90°/s²`

> ⚠️ **PID is all zeros and FF is commented out.** Priority: uncomment `.withFeedforward(armFeedforward())`, tune kG, then add PID.

1. **kG (first!):** Hold arm horizontal. Increase until it holds. YAMS applies `kG × cos(angle)` automatically. (0.21)
2. **kS/kV:** Slow constant-velocity sweep. `kV = (voltage - kG×cos(θ) - kS) / velocity`. Currently kV=0.
3. **kP:** Command stow → deploy. Settle <0.3s. Start at 1.0.
4. **kD:** Significant inertia — expect to need some. Start 0.05.
5. **Profile:** 90°/s is conservative. Increase after PID is stable.

**AdvantageScope:** `IntakePivot/angle`, `IntakePivot/setpoint`

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

Our code publishes telemetry two ways:

| Method | NT Prefix | Used By |
|--------|-----------|---------|
| `Logger.recordOutput("Key", val)` | `/AdvantageKit/Key` | All AdvantageKit-logged data |
| `SmartDashboard.putX("Key", val)` | `/SmartDashboard/Key` | Drive field widget |

In Elastic, when configuring a widget's NT path, use the full prefixed key. For example, `MatchTime` in code → `/AdvantageKit/MatchTime` in Elastic.

### Recommended Match Layout (6328-Style)

6328 (Mechanical Advantage) publishes hub shift data to their dashboard for operator awareness during 2026 Rebuilt's alternating shift windows. Our code does the same via `HubShiftUtil`. Replicate their layout:

#### Row 1 — Match Awareness (biggest widgets, top of screen)

| Widget Type | NT Key | Elastic Widget | Notes |
|-------------|--------|----------------|-------|
| Match Timer | `/AdvantageKit/MatchTime` | **Number** (large font) | Countdown from FMS. Most important number. |
| Shift Timer | `/AdvantageKit/HubShift/ShiftedRemainingTime` | **Number** (large font) | Seconds until current shift ends (TOF-adjusted). |
| Shift Active | `/AdvantageKit/HubShift/ShiftedActive` | **Boolean Box** | Green = our alliance can score. Red = opponent's turn. |
| Game State | `/AdvantageKit/HubShift/ShiftedCurrentShift` | **Text Display** | TRANSITION / SHIFT1 / SHIFT2 / SHIFT3 / SHIFT4 / ENDGAME |
| Active First? | `/AdvantageKit/HubShift/ActiveFirst` | **Boolean Box** | Whether our alliance scores first in SHIFT1. |

#### Row 2 — Shooter Status

| Widget Type | NT Key | Elastic Widget | Notes |
|-------------|--------|----------------|-------|
| Flywheel Ready | `/AdvantageKit/Flywheel/IsSpunUp` | **Boolean Box** | Green = RPM on target. |
| Flywheel RPM | `/AdvantageKit/Flywheel/FX/VelocityRPM` | **Number** | Current flywheel speed. |
| Hood Angle | `/AdvantageKit/Hood/FX/PositionDegrees` | **Number** | Current hood position in degrees. |
| Shoot Mode | `/AdvantageKit/ShooterTelemetry/shootMode` | **Text Display** | FULL / STATIC_DISTANCE / etc. |
| Hub Distance | `/AdvantageKit/ShooterTelemetry/hubDistanceMeters` | **Number** | Distance to hub (meters). |

#### Row 3 — Drive & System Health

| Widget Type | NT Key | Elastic Widget | Notes |
|-------------|--------|----------------|-------|
| Battery | `/AdvantageKit/Battery/Voltage` | **Number** | Flash red < 11.5V. |
| Vision Has Target | `/AdvantageKit/Vision/Summary/HasTarget` | **Boolean Box** | Green = seeing AprilTags. |
| Vision Tag Count | `/AdvantageKit/Vision/Summary/TagCount` | **Number** | How many tags are visible. |
| Field View | `/SmartDashboard/Field` | **Field Widget** | 2D robot pose — verify auto worked. |

### Elastic Setup Step-by-Step

1. **Install:** Download from [GitHub releases](https://github.com/Gold872/elastic-dashboard/releases). Run the installer.
2. **Connect:** Launch Elastic → it auto-discovers the robot via NT4 (same as Shuffleboard). In sim, it connects to `localhost`.
3. **Create tabs:** Right-click the tab bar → Add Tab. Create at least:
   - **Teleop** — the match layout above (auto-selected by `Elastic.selectTab("Teleop")` in code)
   - **Auto** — minimal, just field view + auto selector
   - **Disabled** — auto selector, battery, vision status
4. **Add widgets:**
   - Click **+** or drag from the NT tree on the left.
   - Right-click a widget → **Properties** → set the NT topic path (e.g., `/AdvantageKit/HubShift/ShiftedActive`).
   - For Boolean Box: set true-color = green, false-color = red.
   - For Number widgets: increase font size to **48–72pt** (driver is 10+ feet away).
5. **Save layout:** File → Save Layout → commit the `.json` file to the repo so the whole team uses the same layout.
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
