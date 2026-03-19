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

Starter prompt (paste to assistant)
----------------------------------
Copy this exact block into a new assistant session to rehydrate behavior and expectations:

"Project assistant profile: MRT3216 repo. Use YAMS-first command-returning APIs. Prefer `setVelocity(...)` returns a Command. Use `stopHold()` for default closed-loop zero and `stopNow()` for one-shot imperative stops used inside sequences. Provide `followTarget(Supplier)` re-applier for live tuning when supplier-backed YAMS commands do not re-evaluate. Name composed commands with `withName(...)`. Bump commands must be one-shot and not require subsystems. Follow Oblarg command-based best practices: no stored command instances, all motor access through commands, boolean subsystem state exposed as public final Trigger fields, cross-subsystem coordination in ShooterSystem not in individual subsystems. When making edits, validate with `./gradlew.bat build` and run tests if added. If you are unsure about ownership or blocking semantics, ask a clarifying question before changing public APIs."

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

*Last edited: 2026-02-27 — pushed to `mechanisms`.*
