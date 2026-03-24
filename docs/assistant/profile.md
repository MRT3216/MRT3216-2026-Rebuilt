## Assistant Profile — MRT3216 Command-Based + YAMS Conventions (Expanded)

Purpose
-------
This file is the **primary context document** for AI assistants working on this FRC robot codebase.

**Project**: FRC Team 3216's 2026 robot code — Java, WPILib 2026, YAMS motor framework, AdvantageKit logging, Phoenix Pro (TalonFX/Kraken), REV SparkMax (NEO).
**Branch**: `Claude` (all assistant work happens here)
**Build**: `.\gradlew compileJava` from project root

Quick reference — jump to section:
- **[Planning / execution model split](#planning--execution-model-split)** — read first if you received an execution plan
- **[Core design decisions](#core-design-decisions-concise)** — command patterns, stop semantics
- **[Important YAMS gotchas](#important-yams-gotchas)** — SparkMax unit system, reboot requirement, etc.
- **[Key files & where to look](#key-files--where-to-look-quick-map)** — file path quick-reference
- **[Library source code access](#library-source-code-access)** — how to read dependency source

What this contains
- Key design decisions and rationale
- Practical examples you can paste into code
- Important repo locations to check first when making changes
- A reusable starter prompt to paste into assistant sessions

Planning / execution model split
---------------------------------
This project uses two tiers of AI assistant:
- **Planner** (expensive model — e.g., Claude Opus): reads codebase, makes design decisions, outputs a structured execution plan.
- **Executor** (cheaper model — e.g., Claude Sonnet, GPT-4o): follows the plan step-by-step, making mechanical edits with no judgment calls.

The plan format is defined in [`docs/assistant/plan-format.md`](plan-format.md). When you receive a plan, follow it literally. If a step fails, report the error and stop — don't improvise.

If you are an executor model and there is **no plan attached**, treat this file as your primary context. Read "Core design decisions", "Key files & where to look", and "Important YAMS gotchas" before making any changes.

Core design decisions (concise)
------------------------------
- YAMS-first APIs: expose command-returning methods for mechanisms (e.g., `setVelocity(AngularVelocity)` and `setVelocity(Supplier<AngularVelocity>)`). Commands should own closed-loop behavior.
- Imperative helpers: keep them private and short (e.g., `applySetpoint(...)`). Provide `stopNow()` one-shot commands for sequences where an immediate non-blocking stop is required.
- Two-stop semantics: `stopHold()` (long-running closed-loop hold zero) vs `stopNow()` (one-shot imperative zero).
- Live tuning: YAMS supplier-backed commands may not re-evaluate a Supplier after scheduling — provide `followTarget(Supplier<AngularVelocity>)` (imperative re-applier) for tuning flows.
- Command ownership: callers (systems/containers) should create longer-lived commands. Keep bump and transient actions as runOnce/no-requirements commands to avoid interrupting pipelines.

YAMS Supported Motor Controllers (from SmartMotorFactory.java — github.com/Yet-Another-Software-Suite/YAMS)
---------------------------------------------------------------------------------------------------------
YAMS supports four `SmartMotorController` wrapper implementations:

| Wrapper class | Motor controller | TorqueCurrentFOC support |
|---|---|---|
| `TalonFXWrapper` | `TalonFX` (CTRE Phoenix 6) | ✅ Full — `VelocityTorqueCurrentFOC`, `PositionTorqueCurrentFOC`, `MotionMagicTorqueCurrentFOC`, `MotionMagicVelocityTorqueCurrentFOC` |
| `TalonFXSWrapper` | `TalonFXS` (CTRE Phoenix 6) | ✅ Full — same FOC requests as TalonFXWrapper |
| `SparkWrapper` | `SparkBase` (REV SparkMAX / SparkFlex) | ❌ No FOC |
| `NovaWrapper` | `ThriftyNova` | ❌ No FOC |

**TorqueCurrentFOC via YAMS**: Use `.withVendorControlRequest(new VelocityTorqueCurrentFOC(0))` (or the relevant FOC request) on `SmartMotorControllerConfig` before building your wrapper. YAMS will then dispatch that request in `setVelocity()` / `setPosition()`. The vendor control request must be a `ControlRequest` whose `.getName()` matches one of the supported switch-case entries.

**FOC commutation (EnableFOC flag) via YAMS**: YAMS enables FOC commutation automatically based on the `DCMotor` constant you pass to the wrapper constructor. If you pass a FOC-capable motor (e.g., `DCMotor.getKrakenX60Foc(2)`, `DCMotor.getKrakenX44Foc(1)`), YAMS sets `EnableFOC = true` on the underlying `VelocityVoltage` / `MotionMagicVoltage` requests — giving you ~15% more peak power with no gain changes. This is **separate from and cheaper than** switching to `VelocityTorqueCurrentFOC`. Confirmed by nstrike (YAMS author) in Discord: *"The only middle ground i could do is if u supplied FOC motors as your DCMotor — then i could optionally enable FOC."*

**This project's current state**: `FlywheelSubsystem` uses `DCMotor.getKrakenX60Foc(2)` and `HoodSubsystem` uses `DCMotor.getKrakenX44Foc(1)` → both already have FOC commutation enabled via YAMS. Neither uses `VelocityTorqueCurrentFOC` (torque-current control mode), which would require retuning gains.

> **Note:** YAMS itself **does not** require Phoenix Pro — it will call `setControl()` with whatever request you provide. But TorqueCurrentFOC will only work on a licensed device; an unlicensed TalonFX/TalonFXS will disable output and set the `UnlicensedFeatureInUse` fault.

YAMS API Reference (from official docs — yagsl.gitbook.io/yams)
---------------------------------------------------------------
FlyWheel (velocity mechanisms):
- `flywheel.run(AngularVelocity)` → returns a RunCommand (continuous, never ends on its own). This is the canonical YAMS velocity command. Our `setVelocity()` wraps this.
- `flywheel.getSpeed()` → returns current mechanism AngularVelocity (after gear reduction).
- `flywheel.setMechanismVelocitySetpoint(AngularVelocity)` → imperative setpoint, no command. Use sparingly; prefer `run()`.
- `flywheel.set(double dutyCycle)` → returns RunCommand for open-loop duty cycle control.
- `flywheel.sysId(Volts, Volts/s, Seconds)` → built-in sysId routine available on all mechanisms.
- `periodic()` MUST call `mechanism.updateTelemetry()` — YAMS telemetry does NOT auto-update.
- `simulationPeriodic()` MUST call `mechanism.simIterate()` — required for sim physics to update.

Arm / positional mechanisms:
- `arm.run(Angle)` → RunCommand, holds at angle continuously (does not end at setpoint).
- `arm.runTo(Angle, Angle tolerance)` → ends the command when at setpoint, but does NOT stop the closed-loop controller.
- `arm.setMechanismPositionSetpoint(Angle)` → imperative setpoint.
- `arm.set(double dutyCycle)` → open-loop duty cycle RunCommand.
- `arm.sysId(...)` → same as FlyWheel.

Elevator:
- `elevator.run(Distance)` / `elevator.runTo(Distance, tolerance)` / `elevator.set(double)` → same pattern as Arm.

SmartMotorControllerConfig key options:
- `.withGearing(new MechanismGearing(GearBox.fromReductionStages(a, b)))` → rotor-to-mechanism ratio.
- `.withGearing(double)` → shorthand for simple integer reductions (e.g., `.withGearing(12)` for 12:1).
- `.withMechanismCircumference(Distance)` → for linear mechanisms (elevator, etc.), converts rotations to meters.
- `.withClosedLoopController(kP, kI, kD)` or `.withClosedLoopController(kP, kI, kD, maxVel, maxAccel)` → with optional motion profile.
- `.withSimClosedLoopController(...)` → sim-only PID override (can differ from real PID).
- `.withFeedforward(SimpleMotorFeedforward / ArmFeedforward / ElevatorFeedforward)` → real feedforward.
- `.withSimFeedforward(...)` → sim-only feedforward override.
- `.withTelemetry("Name", TelemetryVerbosity.HIGH)` → required for SmartDashboard/NT telemetry.
- `.withLooselyCoupledFollowers(SmartMotorController...)` → for motors on independent shafts that should follow same setpoint.
- `.clone()` → available since 2026.1.2; use when sharing config across multiple motors with minor differences.

Important YAMS gotchas
----------------------
- **ALWAYS create the Mechanism object** even if you only use SmartMotorController directly — the Mechanism constructor re-applies modified config (soft limits, etc.) to the motor. Skipping this means your config changes won't apply.
- `startRun(() -> smc.stopClosedLoopController(), () -> smc.setDutyCycle(x)).finallyDo(() -> smc.startClosedLoopController())` — official pattern for duty cycle override when in CLOSED_LOOP mode.
- The `Subsystem` passed to `SmartMotorControllerConfig(this)` is only used for YAMS-generated commands (e.g., `arm.run()`). If you call `SmartMotorController` methods directly in your own `run(...)` commands, the subsystem requirement comes from your command factory, not YAMS.
- `getRotorVelocity()` on **SparkWrapper** is BUGGY in the version used by this project — reads position instead of velocity. Derive motor velocity as `mechanismVelocity × kGearReduction` instead. `TalonFXWrapper` and `TalonFXSWrapper` read `getRotorVelocity()` directly from the Phoenix 6 StatusSignal and are not affected by this bug.
- AdvantageKit serializes `AngularVelocity` as rad/s regardless of the unit used to create it. AdvantageScope always displays in rad/s. 700 RPM ≈ 73.3 rad/s. Similarly, `Angle` is serialized in radians.
- SparkMAX persists configuration to flash. After code changes, a robot **reboot** is required to re-apply new YAMS config. Stale flash params cause unexpected behavior (e.g., wrong speed).
- **SparkMax PID/FF unit system with YAMS**: YAMS sets `positionConversionFactor = rotorToMechanismRatio` (= 1/gearing) and `velocityConversionFactor = (1/gearing)/60`. This makes the SparkMax's internal PID see position in **mechanism rotations** and velocity in **mechanism rot/s**. Consequently:
  - **kP** is in Volts per mechanism **rotation** of error (not degrees!). For a 27:1 turret, kP=100 gives ~0.83V at 3° error.
  - **kV** is in Volts per mechanism rot/s. For a NEO through 27:1, theoretical kV ≈ 3.42 V/(mech rot/s).
  - **kS** is in Volts (no unit conversion needed).
  - **kA** is in Volts per (mechanism rot/s²).
  - When using trapezoid profile, YAMS sets `ControlType.kMAXMotionPositionControl` — all PID + FF runs **on the SparkMax hardware**, not the RoboRIO. RIO-side closed-loop thread (Notifier) is only used for exponential profiles or LQR.

YAMS version notes (latest: 2026.1.17 as of 2026-03-19):
- 2026.1.16: Fixed non-profiled closed-loop control in SparkMax, TalonFX, TalonFXS. If using non-profiled PID (no maxVel/maxAccel), ensure using ≥2026.1.16.
- 2026.1.2: `SmartMotorControllerConfig.clone()` added; loosely coupled followers added.
- 2025.12.22: AdvantageKit example added; improved SMC telemetry.
- 2025.10.29: Fixed ArmFeedforward velocity calculation (Rotations→Radians) for Spark/Nova.
- 2025.10.27: `Shooter` renamed to `FlyWheel`.

Library source code access (use `github_repo` tool — DO NOT decompile JARs)
---------------------------------------------------------------------------
The assistant has **direct access** to source code of all major dependencies via the `github_repo` tool. When you need to read library internals (e.g., how YAMS passes feedforward to SparkMax, or how PathPlanner builds auto commands), use `github_repo` with the repo name below — **do NOT extract/decompile JARs from the Gradle cache**.

| Library | GitHub Repo | Key source paths |
|---|---|---|
| **YAMS** | `Yet-Another-Software-Suite/YAMS` | `yams/java/yams/motorcontrollers/local/SparkWrapper.java`, `SmartMotorController.java`, `SmartMotorControllerConfig.java`, `yams/java/yams/mechanisms/positional/Pivot.java` |
| **PathPlanner** | `mjansen4857/pathplanner` | `pathplannerlib/src/main/java/com/pathplanner/lib/` |
| **AdvantageKit** | `Mechanical-Advantage/AdvantageKit` | `junction/core/src/`, `junction/autolog/src/` |
| **PhotonVision** | `PhotonVision/photonvision` | `photon-lib/src/main/java/org/photonvision/` |
| **REVLib** | `REVrobotics/REV-Software-Binaries` | (binary-only; use REV docs instead) |
| **CTRE Phoenix 6** | `CrossTheRoadElec/PhoenixFRC-Releases` | (binary-only; use CTRE docs instead) |
| **WPILib** | `wpilibsuite/allwpilib` | `wpimath/src/main/java/edu/wpi/first/math/`, `wpilibj/src/main/java/edu/wpi/first/wpilibj/` |
| **EasyCRT** | `Yet-Another-Software-Suite/YAMS` | `yams/java/yams/units/EasyCRT.java`, `EasyCRTConfig.java` |

**Usage example**: `github_repo(repo="Yet-Another-Software-Suite/YAMS", query="SparkWrapper setPosition trapezoid profile")`

Why these conventions
- Predictability: command-returning APIs align with WPILib scheduler ownership expectations.
- Safety: one-shot stops avoid deadlocking sequences that need to progress; `stopHold()` provides a stable idle state.
- Tuning ergonomics: follow-target re-appliers and no-requirements bumps let operators adjust without interrupting feeding/aiming.

Key files & where to look (quick map)
--------------------------------------
- Shooter high-level: `src/main/java/frc/robot/systems/ShooterSystem.java`
- Flywheel: `src/main/java/frc/robot/subsystems/shooter/FlywheelSubsystem.java`
- Kicker/Spindexer: `src/main/java/frc/robot/subsystems/shooter/KickerSubsystem.java` and `SpindexerSubsystem.java`
- Turret & Hood: `src/main/java/frc/robot/subsystems/shooter/TurretSubsystem.java` and `HoodSubsystem.java`
- Shooter constants: `src/main/java/frc/robot/subsystems/shooter/ShooterConstants.java` (FlywheelConstants, SpindexerConstants, KickerConstants, HoodConstants, TurretConstants, ShooterModel)
- Shooter lookup table: `src/main/java/frc/robot/subsystems/shooter/ShooterLookupTables.java`
- Intake subsystems: `src/main/java/frc/robot/subsystems/intake/IntakeRollersSubsystem.java` and `IntakePivotSubsystem.java`
- Intake constants: `src/main/java/frc/robot/subsystems/intake/IntakeConstants.java` (Rollers, Pivot)
- Global constants: `src/main/java/frc/robot/constants/Constants.java` (runtime flags, tuningMode)
- Field geometry: `src/main/java/frc/robot/constants/FieldConstants.java`
- Hardware CAN IDs: `src/main/java/frc/robot/constants/RobotMap.java`
- Controller wiring: `src/main/java/frc/robot/RobotContainer.java`

Practical code examples
-----------------------
- Named composed shooting pipeline:
```java
return Commands.parallel(
    flywheel.setVelocity(target),
    Commands.sequence(clearKicker(), spindexer.feedShooter().alongWith(kicker.feedShooter()))
).withName("FeedAndShoot");
```

- One-shot bump that doesn't require subsystems:
```java
public Command bumpFlywheelTarget(AngularVelocity delta) {
  return Commands.runOnce(() -> {
    // update atomic target
  }).withName("BumpFlywheelSys");
}
```

- Follow-target re-applier used for tuning:
```java
public Command followTarget(Supplier<AngularVelocity> supplier) {
  return Commands.run(() -> applySetpoint(supplier.get()), this).withName("FlywheelFollowTarget");
}
```

WPILib + Oblarg Command-Based Best Practices (from ChiefDelphi + frc-docs)
--------------------------------------------------------------------------
Sources: https://www.chiefdelphi.com/t/command-based-best-practices-for-2025-community-feedback/465602
         https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html

COMMANDS:
- Never manually schedule commands (use Triggers or command compositions instead).
- Never store command instances as fields/members — each use should get a fresh instance from a factory method.
- Use factory methods on subsystems to return new Command instances. Do NOT put "Factory" in the method name (e.g., `setVelocity(speed)` not `setVelocityFactory()`). Suffix with `Command` only if disambiguation is needed (e.g., `runIntakeCommand()`).
- Single-subsystem commands → instance factory methods on the subsystem class (`this.run(...)`, `this.startEnd(...)`).
- Multi-subsystem commands → static or instance factory methods on a separate `ShooterSystem` / auto routines class.
- Do NOT put multi-subsystem command logic in a single subsystem class (causes circular dependencies and race conditions).
- Command groups (`.alongWith`, `.andThen`, `Commands.parallel`, `Commands.sequence`) block ALL involved subsystems for their entire duration — default commands do NOT run during a composition. Use `Trigger` for loose coupling instead when this is too rigid.
- Use `.withInterruptBehavior(Command.InterruptBehavior.kCancelIncoming)` to protect critical sequences (e.g., handoffs, collision avoidance).
- Always `.withName(...)` composed commands for readable scheduler traces.

SUBSYSTEMS:
- All access to protected hardware/state MUST go through Commands — no public imperative methods that directly write to motors/actuators (or if present, clearly document the concurrency risk).
- Expose boolean states as `public final Trigger` instance fields for easy loose coupling:
  ```java
  public final Trigger atSpeed = new Trigger(() -> Math.abs(getVelocity().in(RPM) - setpoint.in(RPM)) < 50);
  ```
- Do NOT inject references to other subsystems into a subsystem constructor. Cross-subsystem coordination belongs in `ShooterSystem` / `RobotContainer`.
- `subsystem.periodic()` is acceptable for telemetry, odometry updates, and YAMS `updateTelemetry()` — but motor outputs should be driven by commands, not periodic. Avoid full state machines in `periodic()`.

TRIGGERS:
- `Trigger` represents any boolean robot state, not just buttons (sensor readings, at-speed checks, game states, etc.).
- Prefer `Trigger` over tight command group composition when two subsystems interact but don't need to be simultaneous mutex-holders:
  ```java
  new Trigger(m_feeder::hasGamepiece)
      .and(controller.rightTrigger())
      .whileTrue(flywheel.spinup().alongWith(turret.aim()))
      .and(flywheel::atSpeed).and(turret::onTarget)
      .whileTrue(spindexer.feedShooter());
  ```
- Triggers do NOT need to be stored as variables if immediately bound — the event loop holds the reference.
- Declare reusable triggers as `public final Trigger` fields in the narrowest scope that has the relevant state (usually the subsystem itself).

PROJECT STRUCTURE (WPILib canonical):
- `Robot.java` — minimal; only `CommandScheduler.getInstance().run()` in `robotPeriodic()`.
- `RobotContainer.java` — all subsystem instantiation, button bindings, auto selector. Subsystems are `private final` fields (not globals/singletons).
- `ShooterSystem.java` (or similar) — multi-subsystem command factories, injected with subsystem instances via constructor (the "non-static factory" pattern).
- `Constants.java` — `public static final` grouped by subsystem in inner classes. Use `import static` to avoid verbose prefixes.
- Commands are factory methods, not stored instances.

OBLARG'S RULES (direct from ChiefDelphi post, verbatim):
1. Commands should not be manually scheduled.
2. Commands should not be stored as instances/members.
3. Commands should be returned from factory functions.
4. Commands should not be composed too deeply across subsystems — use Triggers for loose coupling.
5. Subsystems should only expose protected state interactions through Command factories.
6. Subsystems should expose boolean state as public final Trigger members.
7. Subsystems should not receive injected references to other subsystems.
8. Triggers should represent general robot states (not just buttons).
9. Triggers should be used to coordinate cross-subsystem interactions when compositions would be too rigid.
10. Triggers should live as public final fields in the narrowest scope with access to the relevant state.

AdvantageKit IO Interface Pattern (from docs.advantagekit.org)
--------------------------------------------------------------
AdvantageKit's key value: logs ALL inputs on every loop cycle. On replay, the simulator re-runs robot code with logged inputs — so you can add new log fields or modify pipelines post-hoc and see exactly what would have happened.

Standard subsystem structure with IO layers:
1. **IO Interface** (`SubsystemIO`) — defines an `updateInputs(SubsystemInputs inputs)` method + default no-op implementations for all output methods (e.g., `setVoltage(double)`, `setSetpoint(double)`).
2. **IO Implementations** — `SubsystemIOReal` (uses vendor libs) and `SubsystemIOSim` (uses WPILib simulation classes). Switch in `RobotContainer` constructor based on `Robot.isReal()`.
3. **Inputs class** — a flat POJO annotated with `@AutoLog` that holds all sensor readings as primitives/WPILib structs. `@AutoLog` auto-generates `toLog()`/`fromLog()` methods.
4. **Subsystem class** — holds `io` and `inputs` fields. Each cycle: `io.updateInputs(inputs); Logger.processInputs("Name", inputs);`. All control logic reads from `inputs.*`, not directly from the IO layer.

Key rules:
- Outputs (setVoltage, setSetpoint, etc.) are simple methods on the IO interface — no logging needed for outputs.
- Inputs flow through the `inputs` struct ONLY. Never read hardware directly in the subsystem's control logic — only via `inputs.*`.
- Use anonymous IO classes (`new SubsystemIO() {}`) in sim to get all-zero no-op hardware behavior.
- Timestamp/latency compensation: since all inputs are captured atomically per cycle, latency compensation with odometry works correctly.

Example (condensed):
```java
// In subsystem constructor:
io.updateInputs(inputs);
Logger.processInputs("Flywheel", inputs);

// Control logic uses inputs:
double error = inputs.velocityRadPerSec - setpointRadPerSec;
```

SysId / Feedforward Gain Reference (from docs.wpilib.org)
---------------------------------------------------------
Three WPILib feedforward classes (all use `setVoltage()` for physical accuracy):
- `SimpleMotorFeedforward(kS, kV, kA)` — flywheels, drivetrain, turrets, horizontal sliders. Model: V = kS·sgn(v̇) + kV·v̇ + kA·v̈
- `ElevatorFeedforward(kS, kG, kV, kA)` — vertical elevators. Model adds constant kG term.
- `ArmFeedforward(kS, kG, kV, kA)` — arms. Model: V = kG·cos(θ) + kS·sgn(ω) + kV·ω + kA·α

Gain meanings:
- `kS` (V) — static friction; must be measured empirically (SysId). Cannot be modeled.
- `kV` (V·s/unit) — velocity coefficient. Can be estimated from motor free speed.
- `kA` (V·s²/unit) — acceleration coefficient. Often zero for low-inertia mechanisms.
- `kG` (V) — gravity compensation for elevators (constant) or arms (times cos(θ)).

SysId workflow: deploy routine → run quasistatic + dynamic tests in both directions → load `.wpilog` in SysId app → read kS, kV, kA, kG → plug into feedforward + PID controller. YAMS has a built-in `sysId()` routine.

Important: always use `motor.setVoltage()`, not `motor.set()`, when applying feedforward — `set()` doesn't compensate for battery sag.

PathPlanner 2026 Key Notes (from pathplanner.dev)
-------------------------------------------------
- `AutoBuilder.configure(getPose, resetPose, getRobotRelativeSpeeds, drive, controller, config, allianceFlip, this)` — configure in drive subsystem constructor.
- `RobotConfig.fromGUISettings()` — loads robot mass, MOI, wheel positions from PathPlanner GUI. Store result in Constants.
- `PPHolonomicDriveController(translationPID, rotationPID)` — swerve controller. Typical starting gains: kP=5.0 for both translation and rotation.
- Always load autos at startup (in `RobotContainer` constructor), not in `getAutonomousCommand()`. Complex autos cause long delays if loaded at enable time.
- `AutoBuilder.buildAutoChooser()` — auto-populates `SendableChooser` with every auto in deploy dir. Add `deleteOldFiles = true` to `build.gradle` to avoid stale auto options.
- Hot reload: paths can be updated and regenerated without redeploying code.
- `PathPlannerAuto` is a Command — schedule it directly.
- **Named commands** are registered with `NamedCommands.registerCommand("name", command)` before auto routines are built. Essential for event markers in GUI paths.
- Alliance mirroring: PathPlanner mirrors paths to red side automatically. Origin always stays blue side. If you need additional color transforms, do them yourself.

PhotonVision / Vision Integration Notes (from docs.photonvision.org)
--------------------------------------------------------------------
Core API:
- `PhotonCamera(cameraName)` — one instance per physical camera. Name must match PhotonVision UI nickname.
- `camera.getAllUnreadResults()` — preferred over `getLatestResult()` for pose estimation; processes all queued frames for latency accuracy.
- `result.hasTargets()` — ALWAYS check before getting targets in Java/C++ to avoid NPE.
- `result.getBestTarget()` — best target per configured sort criteria.
- `target.getFiducialId()` — AprilTag ID.
- `target.getPoseAmbiguity()` — lower is better; filter out high-ambiguity readings (>0.2 threshold common).
- `target.getBestCameraToTarget()` — Transform3d in camera space (X=forward, Y=left, Z=up).

Pose estimation with `PhotonPoseEstimator`:
- Create one `PhotonPoseEstimator` per camera.
- Constructor: `new PhotonPoseEstimator(fieldLayout, robotToCamTransform)`.
- Strategy priority: first try `estimateCoprocMultiTagPose()` (best accuracy, requires multi-tag enabled in UI), fall back to `estimateLowestAmbiguityPose()` for single-tag.
- Returns `Optional<EstimatedRobotPose>` — empty if no tags seen, solver failed, or ambiguity too high.
- Feed result into drivetrain: `drivetrain.addVisionMeasurement(estPose.toPose2d(), estTimestamp, stdDevs)`.
- Scale standard deviations (`stdDevs`) with distance and tag count — farther/fewer tags → larger stdDevs (less trust).
- New 2025+ strategy: `estimateConstrainedSolvepnpPose()` — constrains robot to be flat on floor for improved accuracy; requires heading data via `addHeadingData()` each frame.

Camera transform: `new Transform3d(new Translation3d(x, y, z), new Rotation3d(roll, pitch, yaw))` where units are meters and radians.

Nuanced Best Practices from Community Discussion (narmstro/viggy96/spacey_sooty, Sep 2024)
------------------------------------------------------------------------------------------
Additional patterns gleaned from the "best practices example" thread:

1. **`runOnce().andThen(waitUntil(condition))` pattern** for positional commands that wait to complete:
   ```java
   public Command setGoal(double meters) {
     return runOnce(() -> feedback.setGoal(meters))
         .andThen(Commands.waitUntil(feedback::atGoal))
         .withName("ElevatorSetGoal");
   }
   ```
   *(Don't use `.until()` on an `InstantCommand` — it ends before the condition can become true.)*

2. **Named setpoints > generic `setGoal`**: wrap `setGoal()` with semantic methods:
   ```java
   public Command toSpeaker() { return setGoal(Constants.SPEAKER_HEIGHT); }
   public Command toAmp()     { return setGoal(Constants.AMP_HEIGHT); }
   ```
   Or use an enum: `public Command moveTo(ElevatorState state) { return setGoal(state.heightM); }`

3. **`asProxy()` for large sequences**: if a command group includes a subsystem but earlier steps don't need to block it, use `.asProxy()` on the subsystem's command so the sequence doesn't inherit its requirement.

4. **`BooleanSupplier` vs `Trigger` as public fields**: for boolean states, use `public final Trigger` directly (same API). For non-boolean sensor values (e.g., pose, velocity), expose as `public final Supplier<T>` fields.

5. **`periodic()` for motor output is OK IF** the subsystem clearly guards access and you use disable flags controlled by commands (e.g., `manualControl` boolean field toggled by command `initialize()`/`end()`). Pure command-execute approach is preferred but periodic is acceptable with discipline.

6. **`RobotContainer` vs `Robot.java` for bindings**: WPILib is moving toward putting bindings in `Robot.java` directly, which makes `TimedRobot.addPeriodic()` easier to use. `RobotContainer` is still accepted but watch for deprecation.

7. **`atGoal` Trigger field example**:
   ```java
   public final Trigger atGoal = new Trigger(feedback::atGoal);
   public final Trigger atSpeed = new Trigger(() -> getVelocity().isNear(setpoint, Percent.of(5)));
   ```

Phoenix 6 Pro — TalonFX Control, StatusSignals, and Configuration
------------------------------------------------------------------
Sources: v6.docs.ctr-electronics.com  (status-signals, configuration, control-requests, talonfx-control-intro, closed-loop-requests)
**Project uses Phoenix Pro (team has license). All devices are on a CANivore.**

### Control Output Types
Three base output types, named `{ClosedLoopMode}{OutputType}` (e.g., `VelocityVoltage`, `VelocityTorqueCurrentFOC`):
- **DutyCycle** — proportion of supply voltage (−1.0 → +1.0). Simple but battery-dependent.
- **Voltage** — compensates for supply voltage; more stable than DutyCycle. Use this as the default.
- **TorqueCurrentFOC** *(Pro required)* — FOC commutation directly controlling torque/current (i.e., acceleration). ~15% more peak power than non-FOC. Advantages:
  - `kV` is generally unnecessary (0 torque = constant velocity with no external forces).
  - `kA` tunable independently of other gains.
  - Gains are in force/torque units, more physically meaningful.
  - Falls back to non-FOC commutation automatically if device is unlicensed; `TorqueCurrentFOC` (without fallback) will disable output + set `UnlicensedFeatureInUse` fault if unlicensed.

FOC can also be enabled for non-torque requests via the `EnableFOC` field:
```java
new VelocityVoltage(0).withEnableFOC(true)  // Voltage + FOC commutation (Pro)
```

Key control request classes for swerve:
| Request | Type | Notes |
|---|---|---|
| `VelocityVoltage` | velocity closed-loop | default for drive wheels |
| `VelocityTorqueCurrentFOC` | velocity closed-loop | Pro; for drive with torque-current |
| `PositionVoltage` | position closed-loop | default for turn motors |
| `MotionMagicVoltage` | profiled position | turn motors; profiled for smoothness |
| `MotionMagicTorqueCurrentFOC` | profiled position, Pro | turn motors with TorqueCurrentFOC |
| `TorqueCurrentFOC` | open-loop torque | drive open-loop characterization |
| `DutyCycleOut` | open-loop duty cycle | simple open-loop |
| `NeutralOut` | neutral | coast/brake per config |

Control output type for AK swerve template is set by `kSteerClosedLoopOutput` and `kDriveClosedLoopOutput` in `TunerConstants.java`:
```java
// Switch to TorqueCurrentFOC for drive (Pro only; requires re-tuning gains):
public static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.TorqueCurrentFOC;
```
> **Note: TorqueCurrentFOC requires different gains than voltage control. Re-tune after switching.**

### Control Request API
```java
// Declare as member field (reuse to avoid GC pressure)
private final VelocityVoltage driveRequest = new VelocityVoltage(0);
private final PositionVoltage steerRequest = new PositionVoltage(0);

// Apply in periodic (setControl is lightweight, not blocking)
driveTalon.setControl(driveRequest.withVelocity(RotationsPerSecond.of(targetRPS)));
steerTalon.setControl(steerRequest.withPosition(Rotations.of(targetRot)));

// One-shot (no periodic auto-resend): set UpdateFreqHz=0 and call setControl() in periodic manually
driveRequest.UpdateFreqHz = 0;  // one-shot mode — YOU must call setControl() periodically
```

### Closed-Loop Gain Slots
- TalonFX supports multiple gain slots (0, 1, 2). Select with `.withSlot(n)` on the control request.
- Configured via `Slot0Configs`, `Slot1Configs`, `Slot2Configs` inside `TalonFXConfiguration`.
- **`GravityType`** in slot config: `Elevator_Static` or `Arm_Cosine` — adds kG feedforward automatically.
- **`StaticFeedforwardSign`**: `UseVelocitySign` (for velocity loops and motion-profiled position) vs `UseClosedLoopSign` (unprofiled position loops — use with caution, can dither near target).
- **`ContinuousWrap`**: enable in `ClosedLoopGeneralConfigs` for continuous mechanisms (swerve steer, turrets) — shortest-path position tracking within 1 rotation.

### StatusSignal API
```java
// Cache signal references (do NOT call getVelocity() every loop — allocates)
private final StatusSignal<AngularVelocity> velSignal = motor.getVelocity();
private final StatusSignal<Angle>           posSignal = motor.getPosition();

// Refresh (blocking until data arrives — use in odometry thread, not in periodic directly)
BaseStatusSignal.refreshAll(posSignal, velSignal);

// CANivore Timesync (Pro) — blocks until all signals arrive synchronously
BaseStatusSignal.waitForAll(0.020 /*timeout s*/, posSignal, velSignal, gyroYaw);

// Get typed values (WPILib units)
AngularVelocity vel = velSignal.getValue();
double velRps       = velSignal.getValueAsDouble();  // in canonical units (rotations/sec for velocity)

// Latency compensation (for odometry — compensates for signal age)
double compensated = BaseStatusSignal.getLatencyCompensatedValue(
    motor.getPosition(), motor.getVelocity());

// Set update frequency BEFORE calling optimizeBusUtilization()
BaseStatusSignal.setUpdateFrequencyForAll(250, posSignal, velSignal);  // 250 Hz for odometry
BaseStatusSignal.setUpdateFrequencyForAll(50, currentSignal);          // 50 Hz for telemetry

// Minimize CAN bus — disables all signals NOT explicitly registered with setUpdateFrequency
motor.optimizeBusUtilization();
ParentDevice.optimizeBusUtilizationForAll(driveMotor, steerMotor, cancoder);

// StatusSignalCollection (cleaner batched API)
final StatusSignalCollection signals = new StatusSignalCollection();
signals.addSignals(motor.getPosition(false), cancoder.getPosition(false), pigeon.getYaw(false));
signals.setUpdateFrequencyForAll(Hertz.of(200));
signals.waitForAll(0.010);
```
> The `false` parameter on `getPosition(false)` / `getVelocity(false)` skips auto-refresh when caching — necessary for signals managed by batched `refreshAll` / `waitForAll`.

### Configuration API
```java
// Full config with method chaining (withXxx returns this for chaining)
final TalonFXConfiguration cfg = new TalonFXConfiguration()
    .withMotorOutput(new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.Clockwise_Positive))
    .withCurrentLimits(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(Amps.of(120))
        .withStatorCurrentLimitEnable(true))
    .withSlot0(new Slot0Configs()
        .withKS(0.1).withKV(0.12).withKP(0.1));

// Clone for leader/follower with different invert
final TalonFXConfiguration followerCfg = cfg.clone()
    .withMotorOutput(cfg.MotorOutput.clone()
        .withInverted(InvertedValue.CounterClockwise_Positive));

// Apply (BLOCKING — do in constructor/init only, never in periodic)
motor.getConfigurator().apply(cfg);

// Factory default (clears all settings)
motor.getConfigurator().apply(new TalonFXConfiguration());

// Read back config (also blocking)
var readCfg = new TalonFXConfiguration();
motor.getConfigurator().refresh(readCfg);
```
> **CRITICAL: `apply()` and `refresh()` are blocking CAN operations. NEVER call in `periodic()`.**

### CANivore Timesync (Pro required)
- All devices on a CANivore auto-synchronize their time bases when Pro-licensed.
- Use `BaseStatusSignal.waitForAll(timeout, signals...)` to block until all synchronized signals arrive together — ideal for swerve odometry.
- All odometry signals (drive position, steer position, gyro yaw) should share the same update frequency for synchronous acquisition (250 Hz on CAN FD, 100 Hz on RIO CAN bus).
- The AK TalonFX swerve template handles this in `PhoenixOdometryThread`.

---

AdvantageKit TalonFX(S) Swerve Template — Architecture Reference
----------------------------------------------------------------
Source: docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template
**This is the drive subsystem architecture we use.**

### Key Files
- `Drive.java` — main drive subsystem. Holds `SwerveDrivePoseEstimator`, PathPlanner `AutoBuilder` config, and `PhoenixOdometryThread`.
- `Module.java` — one instance per swerve module, delegates to `ModuleIO`.
- `ModuleIO.java` — IO interface for one module (drive + steer motor + encoder).
- `ModuleIOTalonFX.java` — real hardware: TalonFX drive, TalonFX steer, CANcoder. **CTRE-only devices.**
- `ModuleIOSim.java` — simulation implementation (gains stored here, NOT in `TunerConstants`).
- `GyroIO.java` / `GyroIOPigeon2.java` / `GyroIONavX.java` — gyro IO layer. Switch in `RobotContainer`.
- `TunerConstants.java` — generated by Tuner X. Contains: module positions, gear ratios, `kSlipCurrent`, `kSpeedAt12Volts`, `kWheelRadius`, PID gains (`steerGains`, `driveGains`), `kDriveClosedLoopOutput`, `kSteerClosedLoopOutput`.
- `PhoenixOdometryThread.java` — dedicated high-frequency odometry thread. Reads signals at 250 Hz (CAN FD) or 100 Hz (RIO CAN); uses `waitForAll()` with CANivore timesync.

### Important Template Gotchas
- **Gear ratio applied in TalonFX firmware** (via `SensorToMechanismRatio`/`RotorToSensorRatio` configs), NOT on the RIO. AK template gains differ from CTRE's default swerve gains for this reason.
- **Do NOT use `Utils.fpgaToCurrentTime()`** for vision timestamps — AK template uses standard FPGA timestamps for `SwerveDrivePoseEstimator`, so PhotonVision/Limelight timestamps can be passed directly to `addVisionMeasurement()` without conversion.
- **Odometry frequency**: stored at top of `Drive.java`. Default: 250 Hz on CAN FD (CANivore), 100 Hz on RIO CAN bus. Customize carefully; monitor CAN bus utilization.
- **Module IO implementations available**:
  - `ModuleIOTalonFX` — TalonFX + TalonFX + CANcoder (default)
  - `ModuleIOTalonFXS` — TalonFXS + TalonFXS + CANdi
  - Can mix/match. Non-Phoenix encoders: use `PhoenixOdometryThread.registerSignal(doubleSupplier)` to add them to the odometry queue.
  - For non-absolute encoders: set `FeedbackSensorSource = RotorSensor`, use `SensorToMechanismRatio` instead of `RotorToSensorRatio`, reset at startup with `tryUntilOk(5, () -> turnTalon.setPosition(encoder.getPositionRotations(), 0.25))`.

### Characterization / Tuning Workflow
1. **Feedforward (drive kS, kV)**: Run "Drive Simple FF Characterization" auto routine — robot slowly accelerates forward. Check console for kS/kV. Copy to `driveGains` in `TunerConstants.java`. No SysId needed for basic use.
2. **Wheel radius**: Run "Drive Wheel Radius Characterization" auto routine on carpet — robot rotates in place. Check console; copy to `kWheelRadius`.
3. **Max speed**: Set `kSpeedAt12Volts` to theoretical max first, drive at full speed, measure actual max in AdvantageScope (`/RealOutputs/SwerveChassisSpeeds/Measured`), update value.
4. **Slip current**: Drive into a wall, watch `/Drive/Module.../DriveCurrentAmps` and velocity. Note current at wheel slip; set `kSlipCurrent`.
5. **PID tuning**: Watch `SwerveStates/Measured` vs `SwerveStates/SetpointsOptimized` in AdvantageScope.
6. **TorqueCurrentFOC**: Set `kDriveClosedLoopOutput = ClosedLoopOutputType.TorqueCurrentFOC` in `TunerConstants`. Re-characterize and re-tune all drive gains (different units). TalonFXS does NOT support TorqueCurrentFOC.

### Vision Integration in Drive Subsystem
- `Drive.addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3,N1> stdDevs)` — call from vision subsystem.
- Timestamp must be in FPGA time (seconds). PhotonVision results use `result.getTimestampSeconds()` — no conversion needed (unlike CTRE swerve library).
- Scale stdDevs: farther/fewer tags → larger stdDevs (less trust in vision).

### Optional Customizations
- **Profiled turning PID**: Replace `PositionVoltage` request with `MotionMagicVoltage` or `MotionMagicTorqueCurrentFOC` in `ModuleIOTalonFX`. Constraints already configured in constructor.
  ```java
  private final MotionMagicVoltage positionVoltageRequest = new MotionMagicVoltage(0.0);
  private final MotionMagicTorqueCurrentFOC positionTorqueCurrentRequest = new MotionMagicTorqueCurrentFOC(0.0);
  ```
- **Swerve setpoint generator** (Team 254 / PathPlanner): Add `SwerveSetpointGenerator` in `Drive`, use in `runVelocity()` for anti-skid control.
- **Real-time thread priority**: Uncomment in `Robot.java` for tighter loop timing — test carefully, can starve NT/CAN threads.
- **maple-sim (Team 5516)**: Full rigid-body swerve simulation library, easily integrates with AK template.
- **PathPlanner**: already configured in `Drive.java` constructor. Tune: robot mass/MOI/wheel coefficient at top of `Drive.java`; drive/turn PIDs in `AutoBuilder`.

---

AdvantageKit Vision Template — Architecture Reference
----------------------------------------------------
Source: docs.advantagekit.org/getting-started/template-projects/vision-template
**This is the vision subsystem architecture we use, combined with the TalonFX swerve template.**

### Features
- Supports both Limelight and PhotonVision out of the box.
- Vision simulation via PhotonLib.
- High-frequency sampling — every observation is processed, none duplicated.
- Advanced filtering with automatic stdDev scaling.
- Detailed filter logging for replay-based tuning.
- Simple targeting (getTargetX) + full pose estimation.
- Deterministic replay.

### Key Files
- `Vision.java` — vision subsystem. Configured primarily via `VisionConstants`.
- `VisionConstants.java` — camera names, robot-to-camera transforms, filter thresholds.
- `VisionIO.java` — IO interface (real vs sim/replay).
- `VisionIOPhotonVision.java` / `VisionIOLimelight.java` — real implementations.
- `VisionIOSim.java` — simulation using PhotonLib `VisionSystemSim`.

### Logging Fields (per camera)
- `TagPoses` — 3D poses of visible tags → use "Vision Target" in AdvantageScope 3D field view.
- `RobotPoses` — raw pose estimates from last cycle → use "Ghost" in AdvantageScope.
- `RobotPosesAccepted` — subset of `RobotPoses` that passed all filters.
- `RobotPosesRejected` — subset removed during filtering.
- `Summary` table — same fields but aggregated across all cameras.

### Limelight 4 / MegaTag 2
- Template publishes robot orientation every loop cycle (required for MegaTag 2 heading-assisted mode).
- For Limelight 4 built-in IMU: configure `imumode_set` key in NT, or import LimelightLib.

### Integration with TalonFX Swerve Template
- Vision `Drive.addVisionMeasurement()` — call from `Vision.periodic()` or via a callback.
- Use FPGA-timestamped results directly (no `Utils.fpgaToCurrentTime()` needed with AK drive template).
- See `getTargetX()` in `Vision.java` and `configureButtonBindings()` in `RobotContainer` for simple targeting example.

---

AdvantageKit 2026 — What's New
------------------------------
Source: docs.advantagekit.org/whats-new

- **Unit Logging**: `@AutoLog` inputs can use WPILib unit types directly (e.g., `public Current current = Amps.of(63.28)`). `Logger.recordOutput("key", value, "unit")` for primitive+unit. `@AutoLogOutput(unit = "inches")` for annotation-based.
- **NetworkTables client logging**: `SystemStats` table auto-logs connected NT clients (dashboards, vision coprocessors) — no user code needed.
- **Improved console logging**: AK now captures exceptions thrown during robot code execution. Console output shown during Replay Watch.
- **3D mechanism logging**: `LoggedMechanism2d.generate3dMechanism()` auto-generates 3D poses from 2D mechanism layouts.
- **Color logging**: WPILib `Color` objects can be logged as hex strings in inputs/outputs.
- **TalonFX(S) swerve template updated**: now includes `ModuleIOTalonFXS` alternative module implementation for TalonFXS + CANdi hardware.
- **Online API docs**: available at docs.advantagekit.org/javadoc.

---

WPILib PID Tuning Walkthrough — Flywheel (Velocity) vs Turret (Position)
------------------------------------------------------------------------
Source: docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-flywheel.html
        docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-turret.html

**Flywheel (velocity control):**
- Pure feedforward works *reasonably well* for velocity — motor steady-state velocity ≈ proportional to voltage. This is why joystick driving "just works."
- Combined FF+PID is ideal: tune FF first (increase kV until output matches setpoint over time), then layer PID on top.
- kD is not useful for velocity with a constant setpoint — only needed when setpoint changes.
- kI is a sub-optimal way to eliminate steady-state error — better to use FF+PID. kI makes control "laggy."
- Bang-bang works for slow-inertia flywheels but causes mechanical stress and current spikes.
- Inertia in velocity control is different from position: shaft stops accelerating when voltage drops to zero (friction + back-EMF slow it), so overshoot is rare (usually only from loop delay).
- kS (static friction) can be important if gearing has high friction. Measure: slowly increase voltage until mechanism moves; kS is the voltage just before motion begins.
- kA is not needed for constant-velocity setpoints — only for changing velocity setpoints.

**Turret (position control):**
- Pure feedforward alone *does not work* for position — feedforward relates voltage to velocity/acceleration, not directly to position. Without a motion profile, it produces a single voltage spike and then zero (the "kick" response).
- Pure PID works *acceptably* for turret position because no control effort is needed to hold position (unlike flywheel). However, it struggles to track a smoothly moving setpoint (no feedforward term to anticipate).
- Combined FF+PID is best: feedforward provides smooth velocity following, PID provides long-term error correction and convergence.
- High static friction turrets get "stuck" near the setpoint when PID output falls below kS — add kS to the feedforward.
- kS direction issue: WPILib SimpleMotorFeedforward uses velocity setpoint for kS sign. Without a motion profile, you must manually add kS based on direction of error.
- **Tuning order**: always tune feedforward first, then PID on top.

**Applicability to our robot:**
- Flywheel, Spindexer, Kicker → velocity control (FF+PID or FF-only for kicker)
- Turret, Hood, Intake Pivot → position control (PID+optional FF)
- All our velocity subsystems use YAMS `SmartMotorFeedforward` + on-controller PID
- Positional subsystems use YAMS motion profiled PID (Hood, Turret) or duty-cycle only (intake pivot — needs retune)

WPILib TrapezoidProfile & ProfiledPIDController Reference
---------------------------------------------------------
Source: docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/trapezoidal-profiles.html
        docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/profiled-pidcontroller.html

**TrapezoidProfile** generates a smooth position/velocity trajectory that respects max velocity and max acceleration constraints. Useful for position mechanisms to avoid "bang" setpoint changes.
```java
// Create profile with constraints
TrapezoidProfile profile = new TrapezoidProfile(
    new TrapezoidProfile.Constraints(maxVel, maxAccel));

// Each loop iteration: calculate next setpoint from current state toward goal
TrapezoidProfile.State setpoint = profile.calculate(kDt, currentState, goalState);

// Feed to a controller
motor.setSetpoint(PIDMode.kPosition, setpoint.position,
    feedforward.calculate(setpoint.velocity) / 12.0);
```

**ProfiledPIDController** wraps TrapezoidProfile + PIDController into one class. You set a *goal* (not a setpoint) — the profile generates intermediate setpoints automatically.
```java
ProfiledPIDController controller = new ProfiledPIDController(
    kP, kI, kD,
    new TrapezoidProfile.Constraints(maxVel, maxAccel));

// In periodic:
double pidOutput = controller.calculate(encoder.getDistance(), goalPosition);
motor.setVoltage(pidOutput + feedforward.calculate(controller.getSetpoint().velocity));
```

**Key points:**
- Goal ≠ setpoint. The goal is the final desired state; the setpoint is the intermediate profiled state.
- Use `controller.getSetpoint()` to get current profiled state (for feedforward calculation).
- Use `controller.atGoal()` to check if the mechanism has reached the final position.
- WPILib feedforward helpers (`SimpleMotorFeedforward.maxAchievableVelocity()` etc.) can calculate safe constraints.
- YAMS handles motion profiling internally via `.withClosedLoopController(kP, kI, kD, maxVel, maxAccel)` — this is equivalent to ProfiledPIDController but runs on the motor controller firmware (lower latency).

**YAMS vs WPILib motion profiling:**
| Feature | YAMS | WPILib |
|---|---|---|
| Where it runs | Motor controller firmware (TalonFX: MotionMagic, Spark: MAXMotion) | roboRIO |
| Latency | ~1ms (firmware loop) | ~20ms (robot loop) |
| Profile type | Trapezoidal (MotionMagic) or S-curve (MotionMagicExpo) | Trapezoidal only |
| Integration | Automatic with `withClosedLoopController(kP,kI,kD,maxVel,maxAccel)` | Manual `ProfiledPIDController` + feedforward |

Elastic Dashboard Reference
----------------------------
Source: github.com/Gold872/elastic-dashboard, frc-elastic.gitbook.io/docs

**What it is:** Modern FRC dashboard (replacement for Shuffleboard). Written in Dart/Flutter. Runs on Windows, macOS, Linux, Web.

**Key features for our team:**
- Customizable color scheme with 20+ variants
- Subscription sharing to reduce NetworkTables bandwidth (important for competition with limited bandwidth)
- Optimized camera streams — automatically deactivate when not in use (saves bandwidth)
- Automatic height resizing to match FRC Driver Station window

**Setup:**
1. Download latest release from GitHub (`v2026.1.2` as of writing)
2. Connect to robot NetworkTables (same as Shuffleboard — IP `10.32.16.2` or mDNS `roboRIO-3216-FRC.local`)
3. Drag widgets from NT tree onto dashboard canvas
4. Use subscription sharing to avoid bandwidth issues at competition

**Java integration (ElasticLib):**
```java
// Send notification to dashboard
Elastic.sendNotification(new Notification(
    NotificationLevel.INFO, "Flywheel Ready", "At speed for shooting"));
```

**Competition tips:**
- Pre-build layouts at home, export/import configs
- Keep camera streams off by default — only enable when operator needs them
- Use subscription sharing mode to minimize NT traffic
- Dashboard auto-recovers after robot code restart
- Test with actual DS at home to verify FMS-compatible bandwidth limits

CTRE Phoenix 6 — Tuner X & Swerve API Reference
-------------------------------------------------
Source: v6.docs.ctr-electronics.com/en/stable/docs/tuner/index.html
        v6.docs.ctr-electronics.com/en/stable/docs/api-reference/mechanisms/swerve/swerve-overview.html

**Phoenix Tuner X** — companion app for CTRE device management:
- Available on Windows (Microsoft Store), Android (Play Store), macOS/iOS (App Store)
- Key features: device list, config editor, self-test snapshot, real-time plotting, multi-device plot & control, swerve project generator, elevator generator
- **Connecting**: USB to roboRIO or network connection. Device list shows all CAN devices.
- **Configs page**: edit all device configurations (PID gains, current limits, etc.) live. Useful for quick tuning at competition.
- **Self-test snapshot**: captures device state at a moment in time — useful for diagnosing issues (shows faults, voltages, currents, temperatures).
- **Plotting**: real-time signal plotting (velocity, position, current, voltage) for any device signal. Multi-device plot & control allows plotting signals from multiple devices simultaneously.
- **Swerve project generator**: auto-generates swerve drivetrain code from device configuration. Our project was generated from this.

**Phoenix 6 Swerve API overview:**
- 5 core classes: `SwerveDrivetrainConstants`, `SwerveModuleConstantsFactory`, `SwerveModuleConstants`, `SwerveDrivetrain`, `SwerveRequest`
- Hardware requirements: 4 TalonFX/FXS drive + 4 TalonFX/FXS steer + 1 Pigeon 2.0 + 4 CANcoders (or PWM encoders via CANdi/FXS)
- All drive motors must be same type; all steer motors must be same type (but drive and steer can differ)
- Built-in `SwerveRequest` types: `FieldCentric`, `RobotCentric`, `FieldCentricFacingAngle`, `SwerveDriveBrake` (X-brake)
- Odometry runs on a separate high-frequency thread, synchronized with motor controller updates
- With CANivore + Pro license + timesync: odometry timestamps are hardware-synchronized for best accuracy
- Simulation: call `updateSimState(dt, supplyVoltage)` in `simulationPeriodic()`
- **`CommandSwerveDrivetrain`** (from CTRE examples / Tuner X generator) integrates cleanly with WPILib command-based framework — our project uses the AdvantageKit variant

WPILib Triggers Deep Dive
--------------------------
Source: docs.wpilib.org/en/stable/docs/software/commandbased/binding-commands-to-triggers.html

**Getting a Trigger:**
1. **HID factories** (most common): `controller.x()`, `controller.rightTrigger()`, etc.
2. **JoystickButton**: `new JoystickButton(controller, Button.kY.value)` — constructor-only subclass of Trigger
3. **Arbitrary**: `new Trigger(limitSwitch::get)` or `new Trigger(() -> someCondition)`

**Binding types:**
| Binding | Schedules when... | Cancels when... | Re-schedules? |
|---|---|---|---|
| `onTrue` | trigger goes false→true | never (runs to completion) | only if trigger cycles false→true again |
| `whileTrue` | trigger goes false→true | trigger becomes false | no (wrap in RepeatCommand for restart) |
| `toggleOnTrue` | trigger goes false→true | same condition if running | alternates schedule/cancel |
| `onFalse`/`whileFalse`/`toggleOnFalse` | same logic but on true→false | — | — |

**Composing triggers:**
```java
// AND: both must be true
controller.x().and(controller.y()).onTrue(new ExampleCommand());
// OR: either must be true
controller.a().or(controller.b()).whileTrue(new AnotherCommand());
// NEGATE: invert
controller.start().negate().onTrue(new StopCommand());
```

**Chaining:** binding methods return the trigger, so you can chain:
```java
exampleButton
    .onTrue(new FooCommand())    // schedule on press
    .onFalse(new BarCommand());  // schedule on release
```

**Debouncing:** `exampleButton.debounce(0.1).onTrue(...)` — avoids rapid re-triggering (useful for digital inputs, noisy sensors).

**Our conventions:**
- Expose subsystem states as `public final Trigger` fields (e.g., `atSpeed`, `onTarget`)
- Use Triggers for cross-subsystem coordination instead of tight command compositions
- Triggers don't need to be stored if immediately bound — the event loop holds the reference
- Define triggers in the narrowest scope that has access to the state

PhotonVision — Constrained Pose Estimation & Heading-Assisted Strategies
------------------------------------------------------------------------
Source: docs.photonvision.org/en/latest/docs/programming/photonlib/robot-pose-estimator.html

**PhotonPoseEstimator setup:**
```java
// Field layout (auto-loads current year's AprilTag layout)
AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

// Camera transform (robot center → camera, meters + radians)
Transform3d robotToCam = new Transform3d(
    new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));

// Create estimator
PhotonPoseEstimator estimator = new PhotonPoseEstimator(tagLayout, robotToCam);
```

**Estimation strategies (9 available):**

| Strategy | Method | Best for | Notes |
|---|---|---|---|
| Coprocessor MultiTag | `estimateCoprocMultiTagPose()` | **Recommended default** — most accurate | Requires field layout in UI |
| Lowest Ambiguity | `estimateLowestAmbiguityPose()` | Single-tag fallback | Simple, reasonable accuracy |
| Closest to Camera Height | `estimateClosestToCameraHeightPose()` | Niche | Height-based selection |
| Closest to Reference Pose | `estimateClosestToReferencePose()` | When you have a good prior | Pass a reference pose |
| Average Best Targets | `estimateAverageBestTargetsPose()` | Multi-tag averaging | Less accurate than coprocessor multitag |
| roboRIO MultiTag | `estimateRioMultiTagPose()` | Legacy — NOT recommended | Slower version of coprocessor multitag |
| PnP Distance Trig | `estimatePnpDistanceTrigSolvePose()` | Single-tag with heading | Needs `addHeadingData()` every frame |
| **Constrained SolvePnP** | `estimateConstrainedSolvepnpPose()` | **Best single-tag** — heading-assisted | Needs `addHeadingData()`, runs on RIO, <2ms |

**Recommended pipeline (from PhotonVision docs):**
```java
for (var result : camera.getAllUnreadResults()) {
    visionEst = estimator.estimateCoprocMultiTagPose(result);       // try multi-tag first
    if (visionEst.isEmpty()) {
        visionEst = estimator.estimateLowestAmbiguityPose(result);  // fallback to single-tag
    }
    updateEstimationStdDevs(visionEst, result.getTargets());
    visionEst.ifPresent(est ->
        drivetrain.addVisionMeasurement(
            est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs));
}
```

**Constrained SolvePnP (heading-assisted):**
- Constrains the robot pose to be flat on the floor (eliminates 3 DOF → much more accurate with single tags)
- Requires calling `addHeadingData()` every robot loop with current gyro heading
- Runs on RoboRIO, typically <2ms compute
- Based on FRC Team 6328's reference implementation
- For best results: feed multi-tag result if available, else feed the single-tag result into `estimateConstrainedSolvepnpPose()`

**Our implementation:** Uses AdvantageKit vision template with `VisionIOPhotonVision`. Processes multiple cameras, adjusts std devs based on tag count and distance, feeds into `SwerveDrivePoseEstimator.addVisionMeasurement()`.

AdvantageScope — Log Analysis Reference
----------------------------------------
Source: docs.advantagescope.org (Mechanical Advantage / FRC 6328)

**What it is:** Free, open-source log viewer for FRC. Works with AdvantageKit `.wpilog` files, DataLog files, and live NetworkTables. Cross-platform (Windows, macOS, Linux, web).

**Key views for debugging:**
| View | Use case |
|---|---|
| Line Graph | Time-series data (velocities, currents, voltages, PID error over time) |
| Table | Raw values at specific timestamps |
| 2D Field | Robot pose, vision measurements, PathPlanner trajectories |
| 3D Field | Same but with 3D mechanism visualization |
| Mechanism | WPILib Mechanism2d visualization |
| Swerve | Module states, desired vs actual vectors |
| Joystick | Controller input visualization |
| Statistics | Min/max/mean/stdev of any signal |
| Console | Printed messages and exceptions |

**Competition workflow:**
1. Deploy with AdvantageKit logging enabled (logs to USB stick on roboRIO)
2. After match: pull USB stick, open `.wpilog` in AdvantageScope
3. Key signals to check first:
   - `Drive/Module*/DriveCurrentAmps` — current draw spikes indicate mechanical issues or stalling
   - `RealOutputs/Robot/BatteryVoltageVolts` — brownout risk if drops below ~7V
   - `Drive/OdometryPose` vs `Vision/*/Pose` — vision agreement with odometry
   - Any subsystem's PID error signals — convergence, oscillation, steady-state error

**Live mode:** Connect to `10.32.16.2:5810` (or mDNS) for real-time telemetry during practice/matches.

**Replay (AdvantageKit exclusive):**
- Load a `.wpilog` from a match, modify robot code (add new logged values, change logic), re-run the log through the modified code
- The replayed log replays exact same inputs but with new processing — lets you debug without re-running on the robot
- Critical for diagnosing issues that only appeared during a match

WPILib NetworkTables Best Practices
------------------------------------
Source: docs.wpilib.org/en/stable/docs/software/networktables/

**What NetworkTables is:** A publish/subscribe key-value store. roboRIO is the server; dashboards, coprocessors, and tools are clients.

**Performance tips:**
- Minimize publishing frequency for non-critical data. Default 100ms is fine for most telemetry; use lower rates for high-frequency data.
- Use `SmartDashboard.putNumber()` / `putBoolean()` for simple dashboard values — wraps NT under the hood.
- For bulk structured data, prefer AdvantageKit's `@AutoLog` or `Logger.recordOutput()` over many individual NT puts.
- Avoid publishing large arrays every loop — NT has bandwidth limits (especially on the field at competition: ~4 Mbps shared with camera streams).
- At competition, disable verbose logging / reduce camera resolution to stay under bandwidth limit.

**Subscription sharing (Elastic Dashboard):** Multiple widgets watching the same NT key share one subscription — reduces bandwidth vs each widget having its own subscription.

**Data types supported:** boolean, int, float, double, string, boolean[], int[], float[], double[], string[], raw byte[], struct (WPILib geometry types like Pose2d).

**Key organization patterns:**
```
/SmartDashboard/Subsystem/Key          — simple telemetry
/AdvantageKit/RealOutputs/Subsystem/... — AdvantageKit structured logging
/photonvision/CameraName/...           — PhotonVision auto-publishes here
/PathPlanner/...                       — PathPlanner trajectory data
```

**Our usage:** AdvantageKit handles most NT publishing automatically. YAMS `.withTelemetry("Name", TelemetryVerbosity.HIGH)` publishes mechanism state. PhotonVision publishes camera data to `/photonvision/`. Elastic Dashboard subscribes to these.

PathPlanner — Path Constraints, Event Markers & Triggers Reference
------------------------------------------------------------------
Source: pathplanner.dev/pplib-build-an-auto.html, pplib-named-commands.html, pplib-triggers.html, pplib-follow-a-single-path.html

**AutoBuilder configuration (swerve):**
```java
AutoBuilder.configure(
    this::getPose,
    this::resetPose,
    this::getRobotRelativeSpeeds,      // MUST be robot-relative
    (speeds, feedforwards) -> driveRobotRelative(speeds),
    new PPHolonomicDriveController(
        new PIDConstants(5.0, 0.0, 0.0),  // translation PID
        new PIDConstants(5.0, 0.0, 0.0)   // rotation PID
    ),
    robotConfig,                        // RobotConfig.fromGUISettings()
    () -> DriverStation.getAlliance()
        .map(a -> a == DriverStation.Alliance.Red).orElse(false),
    this                                // drive subsystem
);
```

**Named Commands** — register BEFORE creating any autos/paths:
```java
NamedCommands.registerCommand("autoBalance", swerve.autoBalanceCommand());
NamedCommands.registerCommand("intake", intake.runIntakeCommand());
NamedCommands.registerCommand("shoot", shooter.shootCommand());
```

**Event Triggers** — alternative to named commands, more flexible:
```java
new EventTrigger("run intake").whileTrue(intake.runCommand());
new EventTrigger("shoot note")
    .and(new Trigger(shooter::atSpeed))
    .onTrue(shooter.feedCommand());
```

**Point Towards Zone Triggers:**
```java
new PointTowardsZoneTrigger("Speaker").whileTrue(turret.aimAtSpeakerCommand());
```

**PathPlannerAuto Triggers** — per-auto event binding:
```java
PathPlannerAuto auto = new PathPlannerAuto("Example Auto");
auto.isRunning().onTrue(Commands.print("Auto started"));
auto.timeElapsed(5).onTrue(Commands.print("5 seconds"));
auto.event("intake").onTrue(intake.runCommand());
auto.activePath("Path 1").onTrue(Commands.print("Following Path 1"));
auto.nearFieldPosition(new Translation2d(2, 2), 0.5)
    .whileTrue(Commands.print("Near scoring position"));
```

**Auto chooser:**
```java
SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();
SmartDashboard.putData("Auto Chooser", autoChooser);
```

**Java warmup** — avoid first-run delay:
```java
public void robotInit() {
    FollowPathCommand.warmupCommand().schedule();  // run in background at startup
}
```

**⚠ Warning:** If an EventTrigger-bound command shares requirements with commands in the auto group, the auto will be interrupted. Keep event-triggered commands on separate subsystems or use `withInterruptBehavior()`.

**⚠ Warning:** Load all autos at code startup, not when auto is enabled. Complex autos have significant load times.

FRC Battery Management Best Practices
--------------------------------------
Sources: ChiefDelphi community discussions, FRC battery best practices documentation

**Battery testing procedure (our team's protocol):**
1. BatteryBeak quick check (voltage + internal resistance)
2. CBA IV discharge at 7.5A to 10.8V — measures capacity
3. BD250 discharge at 20A to 10.8V × 2 cycles — stress test (simulates match loads)
4. Gentle drive test in robot down to 11V — final validation
5. Record all results in team spreadsheet

**Battery health indicators:**
| Metric | Good | Marginal | Replace |
|---|---|---|---|
| BatteryBeak voltage (fully charged) | >12.7V | 12.5-12.7V | <12.5V |
| Internal resistance | <15mΩ | 15-20mΩ | >20mΩ |
| CBA capacity at 7.5A to 10.8V | >17Ah | 15-17Ah | <15Ah |
| Voltage under 20A load (after 1 min) | >12.0V | 11.5-12.0V | <11.5V |

**In-code protections:**
- **Stator current limiting**: limits torque output, prevents motor overheating. Set per-mechanism in YAMS via `withStatorCurrentLimit(amps)`.
- **Supply current limiting**: limits current drawn from battery, prevents brownouts. Set via `withSupplyCurrentLimit(amps)`.
- Typical values: drive 40-60A stator / 40A supply; mechanisms 20-40A stator / 30A supply (tune based on AK match logs).
- Monitor `RealOutputs/Robot/BatteryVoltageVolts` in AdvantageScope — if drops below 8V regularly, increase current limits or test/replace batteries.

**Competition battery management:**
- Bring minimum 6 batteries (ideally 8-10 for a 2-day event)
- Charge immediately after each match — NiMH FRC batteries take ~2 hours for full charge
- Rotate batteries — don't over-use favorites; track match count per battery
- Never deep discharge below 10.5V — damages cells permanently
- Store at room temperature; cold batteries have significantly reduced performance
- Label all batteries with team number, battery number, and purchase date

**Our specific issues (from Flagstaff):**
- Significant voltage drops from: battery mistreatment (deep discharged), insufficient load testing, no current limiting in code
- Fix: automotive load tester + SkyRC BD250 for thorough testing; 14 new Energizer batteries for Idaho; implement current limiting in all subsystems

---

Starter prompt (paste to assistant)
----------------------------------
Copy this exact block into a new assistant session to rehydrate behavior and expectations:

"Project assistant profile: MRT3216 repo. Drive uses AdvantageKit TalonFX swerve template (ModuleIOTalonFX + PhoenixOdometryThread) with Phoenix Pro licensed — CANivore timesync available, TorqueCurrentFOC available. Vision uses AdvantageKit vision template (VisionIOPhotonVision / VisionIOLimelight). Mechanism subsystems (shooter, intake, etc.) use YAMS-first APIs. Use YAMS-first command-returning APIs. Prefer `setVelocity(...)` returns a Command. Use `stopHold()` for default closed-loop zero and `stopNow()` for one-shot imperative stops used inside sequences. Provide `followTarget(Supplier)` re-applier for live tuning. Name composed commands with `withName(...)`. Bump commands must be one-shot and not require subsystems. Follow Oblarg command-based best practices: no stored command instances, all motor access through commands, boolean subsystem state exposed as public final Trigger fields, cross-subsystem coordination in ShooterSystem not in individual subsystems. Use AdvantageKit IO layer pattern (io/inputs separation, @AutoLog). Subsystem constants live next to their subsystems (e.g., `subsystems/shooter/ShooterConstants.java`, `subsystems/intake/IntakeConstants.java`); global constants (runtime flags, field geometry, CAN IDs) stay in `constants/`. Telemetry keys are inlined as private static final String in each subsystem — no centralized TelemetryKeys class. For drive, gains are in TunerConstants.java; gear ratio is applied in TalonFX firmware (SensorToMechanismRatio). For vision, use addVisionMeasurement() on Drive with FPGA timestamps (no Utils.fpgaToCurrentTime() conversion needed). For Phoenix 6 configs, apply() and refresh() are blocking — constructor only, never periodic. For autos, use PathPlanner AutoBuilder configured in drive subsystem; load all autos at startup. When making edits, validate with `./gradlew.bat build` and run tests if added. If you are unsure about ownership or blocking semantics, ask a clarifying question before changing public APIs."

YAMS Deep Reference — SmartMotorControllerConfig & Tuning (from official YAMS presentation by 9658, 6911, and YASS team)
--------------------------------------------------------------------------------------------------------------------

### Motor Types
- **Brushless only** — YAMS does NOT support brushed motors.
- Supported hardware: SparkMax, SparkFlex, ThriftyNova, TalonFX, TalonFXS.
- All brushless motors have an integrated relative encoder.

### SmartMotorController — Four Control Modes
1. **Position** — Angle-based (arms, turrets, wrists) or Distance-based (elevators, linear actuators).
2. **Velocity** — Angle-based or Distance-based.
3. **DutyCycle** — Open-loop, proportion of supply voltage.
4. **Voltage** — Open-loop, compensated for battery sag.

### SmartMotorControllerConfig — Five Config Categories
1. **Motor** — `withControlMode`, `withFollowers`, `withIdleMode`, `withMomentOfInertia`, `withMotorInverted`, `withOpenLoopRampRate`, `withClosedLoopRampRate`, `withStatorCurrentLimit`, `withSupplyCurrentLimit`, `withVoltageCompensation`, `withTemperatureCutoff`
2. **Closed Loop** — `withClosedLoopController(kP, kI, kD [, maxVel, maxAccel])`, `withSimClosedLoopController(...)`, `withFeedforward(SimpleMotorFeedforward | ArmFeedforward | ElevatorFeedforward)`, `withSimFeedforward(...)`, motion profiles (Trapezoidal, Exponential)
3. **Encoder** — `withGearing(MechanismGearing | double)`, `withMechanismCircumference(Distance)`, `withContinuousWrapping(range)`, `withSoftLimit(min, max)`, `withHardLimit(...)`
4. **External Encoder** — `withExternalEncoder(encoder)`, `withExternalEncoderZeroOffset(angle/distance)`. Only same-vendor absolute encoders supported. External gearing only supported if encoder is geared UP (absolute encoder must never require >1 rotation).
5. **Telemetry** — `withTelemetry("Name", TelemetryVerbosity.HIGH/LOW/NONE)`

### Key Motor Config Notes

**Followers**: Only alike-vendor motor controllers can follow a leader. Define with `withFollowers(SmartMotorController...)`, invert individual followers in the same call.

**Moment of Inertia**: Used for more accurate simulation. Composed of mechanism mass and distance from pivot/axis.

**Stator vs Supply Current**:
- *Stator Current* — current seen by the spinning part of the motor (includes motor brake as negative current). Use `withStatorCurrentLimit` to protect against stalling.
- *Supply Current* — current the controller pulls from the 12V bus. Use `withSupplyCurrentLimit` to protect the PDP/PDH.

**Voltage Compensation**: Compensates for bus voltage sag under load. Keeps control loop output constant despite battery droop. Cannot increase voltage beyond what is available — if the actuator is already saturating, you must address mechanism gearing or motor sizing instead.

**Actuator Saturation**: Occurs when gains are too aggressive and the controller demands motion faster than the mechanism can achieve. Results in erratic behavior. Fix by modifying gearing or using a more powerful motor.

**Temperature Cutoff**: Only supported on SparkMax and SparkFlex. Sets output to 0 in closed loop if temperature threshold is exceeded.

**Ramp Rates**: NOT RECOMMENDED — they slow control response. Use stator current limits and motion profiles (Trapezoidal or Exponential) instead. Two types: open-loop (for voltage/dutyCycle) and closed-loop (interpolates setpoint over X seconds).

**Idle Modes**:
- `Brake` — shorts motor wires at 0 power; holds position against moderate torque.
- `Coast` — no braking at 0 power.

### Gearing & Measurement
- `reductionRatio` = number of motor rotations per one mechanism rotation (e.g., 10.0 for 10:1 reduction).
- `withGearing(new MechanismGearing(GearBox.fromReductionStages(a, b)))` — supports multi-stage compound gearboxes.
- Applied in firmware for TalonFX; YAMS reads back mechanism position/velocity (post-gearbox).
- `withMechanismCircumference(Distance)` — for linear mechanisms, converts rotations → meters.
- All position/velocity readings from YAMS are at the **mechanism** level (post-gearbox), not the rotor.

### Encoder Wrapping / Continuous Wrapping
- Use `withContinuousWrapping(range)` for absolute encoders on continuously rotating mechanisms (turrets, swerve steer).
- Valid ranges: `[-0.5, 0.5)`, `[-1, 0)`, `[0, 1)` in Rotations. The PID will take the shortest path across the wrap boundary.
- Note: the mechanism will never actually reach the max boundary value — it wraps just before.

### Soft vs Hard Limits
- **SoftLimit**: Encoder-based digital barrier. Requires correct encoder setup. Applied via YAMS config.
- **HardLimit**: Physical button at mechanism extremes. YAMS simulation prevents movement past hard limits.
- **HardStop**: Physical bumper/metal stop. Last resort if motors are commanded past limits.

### Feedforward Recap (from YAMS presentation)
- `kS` — voltage to overcome static friction. Same regardless of velocity. Use `signum(velocity)`.
- `kV` — voltage to maintain a given constant velocity (counters back-EMF + viscous drag). Nearly perfectly linear for FRC motors.
- `kA` — voltage to produce a given acceleration. Nearly perfectly linear for FRC motors.
- `kG` — voltage to resist gravity (constant for elevators; `kG × cos(θ)` for arms).
- In sim: `kS` will always read as invalid (no friction in sim). Do not worry about it in simulation.

### Motion Profiles
**Trapezoidal Profile** — limits both acceleration and velocity; smoother than raw PID.
- Constraints: `maxVelocity`, `maxAcceleration`.
- Benefits: full control throughout motion, manages inertia at setpoint transitions, improved repeatability under battery/motor load changes.
- Configure via `.withClosedLoopController(kP, kI, kD, maxVel, maxAccel)`.

**Exponential Profile** — better for mechanisms with significant friction the feedforward can't fully overcome.
- Constraints: `kV` (Voltage/AngularVelocity), `kA` (Voltage/AngularAcceleration).
- `kV` determines max profile velocity: max velocity = supplyVoltage / kV. Higher kV → slower max velocity (safer to start high).
- Factory methods:
  - `ExponentialProfilePIDController.createArmConstraints(maxVolts, DCMotor, weight, armLength, gearing)`
  - `ExponentialProfilePIDController.createElevatorConstraints(maxVolts, DCMotor, carriageWeight, height, gearing)`
  - `ExponentialProfilePIDController.createFlywheelConstraints(maxVolts, DCMotor, flywheelWeight, wheelRadius, gearing)`
  - `ExponentialProfilePIDController.createConstraints(maxVolts, maxAngularVelocity_kV, maxAngularAcceleration_kA)`

### Tuning Order (YAMS official)
1. Set kG, kV, kA, kP, kI, kD all to **zero**. Limit max velocity and acceleration to prevent damage (Elevators: 0.3 m/s, 0.3 m/s²; Arms: 90 deg/s, 90 deg/s²).
2. **kG**: Increase until mechanism starts moving, then back off until it stops. Tune to 2–3 decimal places.
3. **kV**: Start at 0.1, increase in larger steps. Match the slope of position graph to reference graph during motion.
4. **kA**: Start at 0.001. Match the acceleration portion of the position graph to the reference. Generally very small.
5. **kP**: Increase until mechanism overshoots, then back off until overshoot stops.
6. **kD** (optional): Increase max velocity/acceleration to full speed first. Add kD to allow kP to be raised higher — mechanism moves fast then "slams the brakes." On lightweight/slow mechanisms, P alone may suffice.
7. **kI** (optional): Error accumulator. Clear the accumulator before enabling kI from zero.
- Note: `kS` is not needed in simulation and rarely needed on well-designed mechanisms. More useful for arms/pivots than elevators.

### SysId via YAMS
- YAMS provides a built-in `sysId()` routine on all mechanisms — runs quasistatic forward/reverse + dynamic forward/reverse automatically.
- **Bind to `.whileTrue()`** — releasing the button stops the test. If you release early, **discard the log and restart**.
- Soft limits must be set correctly — YAMS uses them to know when to reverse direction.
- CTRE users must manually start/stop the CTRE signal logger. Modify the YAMS SysId command to call the logger:
  ```java
  // Wrap with signal logger start/stop for CTRE hoot logs
  mechanism.sysId(volts, voltsPerSec, seconds)
      .beforeStarting(SignalLogger::start)
      .finallyDo(SignalLogger::stop);
  ```
- Hoot log → Phoenix "Log Extractor" → convert to WPILOG → open in SysId app (select: State, MotorVoltage, Velocity, Position for the leader motor; state signal is not stored under any motor).
- REV logs use `.revlog` format; must also be converted.
- In SysId analyzer: change loop type from Velocity to **Position**. Gain preset: **Default** (YAMS handles unit conversions).
- Run SysId in **simulation first** to validate mechanism config before running on real hardware.

### Mixing Closed-Loop and Open-Loop Control
Some motor controllers (especially SparkMAX/SparkFlex) run PID+feedforward on the roboRIO when they lack on-board support. YAMS calls `setVoltage()` every 20ms in closed-loop mode on these devices. This means you **cannot** call `setVoltage()` out-of-band while the closed-loop is running.

To use open-loop control temporarily:
```java
// Command-based pattern:
subsystem.startRun(
    () -> smc.stopClosedLoopController(),
    () -> smc.setDutyCycle(0.5)
).finallyDo(() -> smc.startClosedLoopController());
```

### Simulation & Telemetry — Required Calls
```java
@Override
public void periodic() {
    mechanism.updateTelemetry();  // REQUIRED — telemetry does NOT auto-update
}

@Override
public void simulationPeriodic() {
    mechanism.simIterate();  // REQUIRED — sim physics do NOT update without this
}
```

### CTRE Tuning in AdvantageScope
To tune CTRE motors in AdvantageScope without Phoenix Tuner splitscreen, publish Phoenix 6 signals via AK logging after creating the motor in the subsystem constructor, then add them to `periodic()` (requires AK vendor dep):
```java
// After motor creation in constructor:
BaseStatusSignal.setUpdateFrequencyForAll(50, motor.getPosition(), motor.getVelocity());
motor.optimizeBusUtilization();

// In periodic():
Logger.recordOutput("Subsystem/PositionRot", motor.getPosition().getValueAsDouble());
Logger.recordOutput("Subsystem/VelocityRps", motor.getVelocity().getValueAsDouble());
Logger.recordOutput("Subsystem/ReferenceRot", setpointRotations);
```
Open a line graph in AdvantageScope, drag position + reference to left axis. Click the 3-line button (turns purple) to enable live tuning via AdvantageScope.

### Live Tuning Mode
- Requires **Test Mode** (in Driver Station) for both SIM and REAL.
- In Elastic: add the YAMS Live Tuning widget to toggle on/off.
- In AdvantageScope: use the 3-line button on a graph (turns purple when active) to type in new gain values live.

### External Encoder Zero Offset
1. Ensure absolute encoder is connected and reflected in code.
2. Open Elastic or similar telemetry app, read absolute encoder value.
3. Hold mechanism at its true "zero" position (e.g., arm parallel to floor = 0°, pointing up = 90°).
4. Record the encoder reading at that position.
5. Pass to `.withExternalEncoderZeroOffset(angle/distance)` in config.

If absolute encoder is not compatible with YAMS, periodically seed the relative encoder:
```java
// In periodic() or on enable — position is MECHANISM (post-gearbox), not rotor:
smc.setMechanismPosition(absoluteEncoder.getPosition());
```

---

Useful local references (already in repo)
 - `docs/TechnicalReference.md` — consolidated technical reference (YAMS, AdvantageKit, telemetry, vision, drive).
 - `docs/TuningGuide.md` — comprehensive PID, feedforward, and motion profile tuning procedures for every subsystem.
 - `docs/TUNING_CHECKLIST.md` — GitHub Issue-ready checklist for tracking tuning progress (mirrors TuningGuide sections).
 - `docs/TurretAimPipeline.md` — turret wrap-around logic, encoder alignment, EasyCRT, aim pipeline.
 - `docs/ControllerGuide.md` — driver & operator controls, shooting modes, LED patterns.
 - `docs/TestModeTuning.md` — test-mode tuning procedures and SysId workflow.
 - `docs/README.md` — documentation index.
 - `docs/assistant/profile.md` — this file; conventions, starter prompt, and reference material for assistant sessions.
 - `docs/assistant/history.md` — single ongoing archive of all assistant conversation sessions (chronological).

Competition Status & Pre-Boise Priorities (updated 2026-03-21)
--------------------------------------------------------------

### Post-Flagstaff Summary
- Robot functioned well despite minimal pre-competition testing (shooter calibrated and lookup table built in hotel parking lot).
- Finished **4th highest EPA** at Arizona; currently predicted **5th highest EPA in Idaho**.
- **Highest world EPA ranking ever for the team**: 513th in the world (top 13.6%) as of Flagstaff.
- Won the **Quality Award**. Still in contention for Worlds — likely needs top-5 qualification + Semifinals/Finals run in Boise, or another award.
- Entire competition ran with **turret locked aiming backwards** and still among the most efficient scorers.

### Known Robot Issues & Status

**Intake (primary failure point)**
- Original carbon fiber dead-axle rollers with 3D-printed inserts → inserts broke under hits.
- Second form (churro through CF tube) → CF tubes began snapping.
- Competed on PVC pipe rollers at end of event — worked acceptably.
- **Fix**: Moving to polycarbonate rollers with metal inserts (WCP inserts + axles from Braeden Hunt / Bear Metal 2046). 8ft polycarb from McMaster, 4ft from AndyMark. Backup 3D-printed inserts also prepared.
- Potential "bash bar" mod: polycarbonate maxspline tube on intake front to absorb impact hits (Ben investigating).

**Turret**
- Wiring not functional without a mechanically designed cable chain — wires blocked shots and prevented feeding.
- **Fix**: 3D printing bidirectional cable chain links; Ben also investigating solutions.
- Turret tracking code nearly complete (was intentionally locked for Flagstaff).

**Shooter Wheels**
- Thrifty Bot shooter wheels wore out fast in the center at Flagstaff.
- **Fix**: 8 AndyMark Stealth Wheels in various durometers ordered (arriving Thursday). Need to test for best shooting consistency and rebuild lookup table after wheel swap.

**Spindexer / Hopper**
- Polycarbonate on spindexer jammed in final match.
- Need to test spindexer without the polycarbonate to evaluate feed performance.
- Balls getting caught in front corners of robot — needs a better solution.
- Hopper should ideally retract with intake to prevent ball jams during agitation.

**Electrical / Batteries**
- Significant voltage drops at Flagstaff from: battery mistreatment (deep discharged), insufficient load testing, no current limiting in code.
- **Fix**: Automotive load tester + SkyRC BD250 discharger (20A vs old CBA V at 7.5A) for thorough battery testing.
- 14 new Energizer batteries inbound for Idaho (8 from Braeden Hunt, 6 from parents).
- Battery test procedure: BatteryBeak check → CBA IV discharge at 7.5A to 10.8V → BD250 at 20A to 10.8V × 2 → drive in robot gently to 11V. All recorded in spreadsheet.

**CAN Bus**
- CANivore CAN FD bus issue occurred in the last match at Flagstaff (fixed during match, reappeared at end).
- Must be diagnosed and corrected before Boise. Pigeon also needs recalibration.

### Programming Priorities for Boise (in order)
1. Review AdvantageKit match logs (identify current draws, anomalies)
2. **Implement current limiting** on all subsystems (stator + supply)
3. **Turret tracking code** — nearly complete, needs testing
4. **Rebuild shooter lookup table** with new AndyMark Stealth Wheels + build tweaks
5. **Retune PID and FF** for all subsystems (intake pivot especially — currently has zero PID/kV, only kG+kS gravity comp; needs full retune Monday)
6. Reprint camera mounts (stronger, cameras right-side-up), lower PhotonVision resolution for higher FPS
7. Refine and test autonomous routines (never tested on real robot before Flagstaff — add real-robot validation)
8. **Create Elastic Dashboard** for in-match feedback
9. Add passing settings
10. Fix CANivore / CAN bus issues
11. Clean up wiring
12. Miscellaneous tweaks (agitator timing, etc.)

### Completed This Session (2026-03-21)
- ✅ **Constants reorganized**: subsystem constants moved from `constants/` to subsystem packages (`subsystems/shooter/ShooterConstants.java`, `subsystems/intake/IntakeConstants.java`). Global constants remain in `constants/` (Constants.java, FieldConstants.java, RobotMap.java).
- ✅ **TelemetryKeys.java deleted**: telemetry key strings inlined as `private static final String` in each subsystem.
- ✅ **Dimensions.java deleted** (unused).
- ✅ **Constants consistency pass**: all velocity and positional subsystem constants now follow a standardized section order (Mechanical → Motor wiring → PID → Feedforward → Sim overrides → FF factories → Limits → Targets/tunables).
- ✅ **PID/FF audit**: reviewed all gains. Identified intake pivot as having zero closed-loop control (intentional — team used duty-cycle at first comp, retune planned Monday). Kicker runs pure feedforward (kP=0.0), acceptable for its role.
- ✅ **Documentation consolidated**: legacy `docs/guides/` folder removed, content merged into `docs/TechnicalReference.md`, `docs/OperatorGuide.md`, `docs/TestModeTuning.md`.

### Key Programming Context for This Week
- **Lookup table**: `src/main/resources/shooter_lookup.json` — will need to be rebuilt after new shooter wheels are installed and tested. Distance-to-RPM mapping. Lookup table code in `subsystems/shooter/ShooterLookupTables.java`.
- **Current limiting**: add `withStatorCurrentLimit` and `withSupplyCurrentLimit` to YAMS `SmartMotorControllerConfig` for all TalonFX and SparkMAX/Flex motors. Review AdvantageKit logs first to set appropriate limits. Note: `kStatorCurrentLimit` constants already exist in each subsystem's constants class — wire them into the YAMS configs.
- **Turret tracking**: code "nearly complete" — likely needs to be wired up to vision pose data and tested on real robot with the new cable chain.
- **PhotonVision**: lower resolution setting in PhotonVision web UI (not in robot code). Camera mount reprint may shift extrinsics — re-run camera calibration if angles change.
- **Intake pivot PID**: zero closed-loop gains (kP=0, kV=0). Only gravity comp (kG=0.21, kS=0.11). Retune Monday using YAMS tuning order: kG → kV → kA → kP → kD. All sim overrides also zeroed.
- **Constants organization**: subsystem constants live next to their subsystems (`subsystems/shooter/ShooterConstants.java`, `subsystems/intake/IntakeConstants.java`). Each inner class follows a canonical section order — new gains from retuning should be placed in the matching section.

Backup & transcript policy
-------------------------
- All session transcripts are appended chronologically to `docs/assistant/history.md` — single file, no per-session archives.
- `docs/assistant/profile.md` holds conventions, reference material, and the starter prompt — update it when conventions change.
- Both files are committed to GitHub so any machine can start a session with full context.
- Avoid committing secrets — redact before committing.

Suggested next improvements (optional)
- Add small JUnit-style integration tests that validate bump commands are non-requiring and triggers start/stop pipelines correctly.
- Add a small `scripts/verify-commands.sh` that runs `./gradlew build` and greps for `withName("` uses to ensure command naming coverage.
- Publish the starter prompt into repository-level VS Code snippets for convenience.

Maintenance
-----------
If you change important conventions (e.g., switch to gating feeding only when at-speed), update this profile and consider adding a migration note in the changelog.

---

*Last edited: 2026-03-24 — added SparkMax PID/FF unit system gotcha (mechanism rotations, not degrees); turret kP=100, kV=3.4 with unit explanation; CRT calibration complete; TuningDashboard created.*
