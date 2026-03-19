## Assistant Profile — MRT3216 Command-Based + YAMS Conventions (Expanded)

Purpose
-------
This file captures an actionable, medium-length summary of the project's command-based & YAMS usage patterns so the assistant and contributors can restore the same conventions on any machine.

What this contains
- Key design decisions and rationale
- Practical examples you can paste into code
- Important repo locations to check first when making changes
- A reusable starter prompt to paste into assistant sessions

Core design decisions (concise)
------------------------------
- YAMS-first APIs: expose command-returning methods for mechanisms (e.g., `setVelocity(AngularVelocity)` and `setVelocity(Supplier<AngularVelocity>)`). Commands should own closed-loop behavior.
- Imperative helpers: keep them private and short (e.g., `applySetpoint(...)`). Provide `stopNow()` one-shot commands for sequences where an immediate non-blocking stop is required.
- Two-stop semantics: `stopHold()` (long-running closed-loop hold zero) vs `stopNow()` (one-shot imperative zero).
- Live tuning: YAMS supplier-backed commands may not re-evaluate a Supplier after scheduling — provide `followTarget(Supplier<AngularVelocity>)` (imperative re-applier) for tuning flows.
- Command ownership: callers (systems/containers) should create longer-lived commands. Keep bump and transient actions as runOnce/no-requirements commands to avoid interrupting pipelines.

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

Important YAMS gotchas:
- **ALWAYS create the Mechanism object** even if you only use SmartMotorController directly — the Mechanism constructor re-applies modified config (soft limits, etc.) to the motor. Skipping this means your config changes won't apply.
- `startRun(() -> smc.stopClosedLoopController(), () -> smc.setDutyCycle(x)).finallyDo(() -> smc.startClosedLoopController())` — official pattern for duty cycle override when in CLOSED_LOOP mode.
- The `Subsystem` passed to `SmartMotorControllerConfig(this)` is only used for YAMS-generated commands (e.g., `arm.run()`). If you call `SmartMotorController` methods directly in your own `run(...)` commands, the subsystem requirement comes from your command factory, not YAMS.
- `getRotorVelocity()` on SparkWrapper is BUGGY in the version used by this project — reads position instead of velocity. Derive motor velocity as `mechanismVelocity × kGearReduction` instead.
- AdvantageKit serializes `AngularVelocity` as rad/s regardless of the unit used to create it. AdvantageScope always displays in rad/s. 700 RPM ≈ 73.3 rad/s.
- SparkMAX persists configuration to flash. After code changes, a robot **reboot** is required to re-apply new YAMS config. Stale flash params cause unexpected behavior (e.g., wrong speed).

YAMS version notes (latest: 2026.1.17 as of 2026-03-19):
- 2026.1.16: Fixed non-profiled closed-loop control in SparkMax, TalonFX, TalonFXS. If using non-profiled PID (no maxVel/maxAccel), ensure using ≥2026.1.16.
- 2026.1.2: `SmartMotorControllerConfig.clone()` added; loosely coupled followers added.
- 2025.12.22: AdvantageKit example added; improved SMC telemetry.
- 2025.10.29: Fixed ArmFeedforward velocity calculation (Rotations→Radians) for Spark/Nova.
- 2025.10.27: `Shooter` renamed to `FlyWheel`.

Why these conventions
- Predictability: command-returning APIs align with WPILib scheduler ownership expectations.
- Safety: one-shot stops avoid deadlocking sequences that need to progress; `stopHold()` provides a stable idle state.
- Tuning ergonomics: follow-target re-appliers and no-requirements bumps let operators adjust without interrupting feeding/aiming.

Key files & where to look (quick map)
- Shooter high-level: `src/main/java/frc/robot/systems/ShooterSystem.java`
- Flywheel: `src/main/java/frc/robot/subsystems/shooter/FlywheelSubsystem.java`
- Kicker/Spindexer: `src/main/java/frc/robot/subsystems/shooter/KickerSubsystem.java` and `SpindexerSubsystem.java`
- Turret & Hood: `src/main/java/frc/robot/subsystems/shooter/TurretSubsystem.java` and `HoodSubsystem.java`
- Controller wiring: `src/main/java/frc/robot/RobotContainer.java`
 - YAMS docs & vendordep: `docs/guides/yams.md` and `vendordeps/yams.json`
 - Telemetry notes: `docs/guides/telemetry.md`

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

Starter prompt (paste to assistant)
----------------------------------
Copy this exact block into a new assistant session to rehydrate behavior and expectations:

"Project assistant profile: MRT3216 repo. Drive uses AdvantageKit TalonFX swerve template (ModuleIOTalonFX + PhoenixOdometryThread) with Phoenix Pro licensed — CANivore timesync available, TorqueCurrentFOC available. Vision uses AdvantageKit vision template (VisionIOPhotonVision / VisionIOLimelight). Mechanism subsystems (shooter, intake, etc.) use YAMS-first APIs. Use YAMS-first command-returning APIs. Prefer `setVelocity(...)` returns a Command. Use `stopHold()` for default closed-loop zero and `stopNow()` for one-shot imperative stops used inside sequences. Provide `followTarget(Supplier)` re-applier for live tuning. Name composed commands with `withName(...)`. Bump commands must be one-shot and not require subsystems. Follow Oblarg command-based best practices: no stored command instances, all motor access through commands, boolean subsystem state exposed as public final Trigger fields, cross-subsystem coordination in ShooterSystem not in individual subsystems. Use AdvantageKit IO layer pattern (io/inputs separation, @AutoLog). For drive, gains are in TunerConstants.java; gear ratio is applied in TalonFX firmware (SensorToMechanismRatio). For vision, use addVisionMeasurement() on Drive with FPGA timestamps (no Utils.fpgaToCurrentTime() conversion needed). For Phoenix 6 configs, apply() and refresh() are blocking — constructor only, never periodic. For autos, use PathPlanner AutoBuilder configured in drive subsystem; load all autos at startup. When making edits, validate with `./gradlew.bat build` and run tests if added. If you are unsure about ownership or blocking semantics, ask a clarifying question before changing public APIs."

Useful local references (already in repo)
 - `docs/guides/yams.md` — vendor install, links to YAMS docs, and licensing notes.
 - `docs/guides/telemetry.md` — AdvantageKit and telemetry guidance used by this project.
 - `docs/assistant/history-2026-02-27.md` — conversation backup created on 2026-02-27 (timestamped archive).

Backup & transcript policy
-------------------------
- Prefer appending user+assistant transcripts to `docs/assistant/history.md` when requested. Avoid committing secrets — redact before committing or instruct the assistant to redact automatically.
- If you want per-session archives, use timestamped files `docs/assistant/history-YYYY-MM-DD.md` (already used).

Suggested next improvements (optional)
- Add small JUnit-style integration tests that validate bump commands are non-requiring and triggers start/stop pipelines correctly.
- Add a small `scripts/verify-commands.sh` that runs `./gradlew build` and greps for `withName("` uses to ensure command naming coverage.
- Publish the starter prompt into repository-level VS Code snippets for convenience.

Maintenance
-----------
If you change important conventions (e.g., switch to gating feeding only when at-speed), update this profile and consider adding a migration note in the changelog.

---

*Last edited: 2026-03-19 — added Phoenix 6 Pro, TalonFX swerve template, vision template, and AK 2026 what's-new sections.*
