# Copilot Instructions — MRT3216 2026 Robot Code

## Project Overview
FRC Team 3216's 2026 robot: Java 17, WPILib 2026, **YAMS** motor framework, **AdvantageKit** logging, **Phoenix 6 Pro** (TalonFX/Kraken on CANivore), **REV SparkMax** (NEO), **PathPlanner** autos, **PhotonVision** pose estimation. Build: `.\gradlew compileJava`. Simulate: `.\gradlew simulateJava`.

## Architecture — Three-Layer Pattern
1. **Subsystems** (`subsystems/<mechanism>/`) — single-mechanism YAMS wrappers. Expose command factories (`setVelocity()`, `setAngle()`, `stopNow()`, `stopHold()`). No cross-subsystem references.
2. **Systems** (`systems/ShooterSystem.java`, `systems/IntakeSystem.java`) — multi-subsystem coordinators. Receive subsystem instances via constructor. Own composed commands (`aimAndShoot`, `hybridAimAndShoot`, `testShoot`).
3. **RobotContainer** — instantiates subsystems/systems, wires button bindings, configures defaults and autos.

## Key Subsystems
- **Drive**: AdvantageKit TalonFX swerve template (`Drive.java`, `Module.java`, `ModuleIOTalonFX.java`). Odometry at 250 Hz via `PhoenixOdometryThread`. Generated config in `generated/TunerConstants.java` — do NOT hand-edit.
- **Shooter**: `FlywheelSubsystem` (Kraken×60 FOC), `TurretSubsystem` (SparkMax NEO 27:1), `HoodSubsystem` (Kraken×44 FOC), `KickerSubsystem`, `SpindexerSubsystem`. All use YAMS `FlyWheel`/`Arm`/`Pivot` abstractions.
- **Vision**: 4 PhotonVision cameras (`VisionIOPhotonVision`); 2 in sim. IO interface pattern per AdvantageKit.
- **Intake**: `IntakePivotSubsystem` + `IntakeRollersSubsystem`, coordinated by `IntakeSystem`.

## Critical Conventions

### Command Patterns (Oblarg's Rules)
- Commands are **factory methods** returning fresh instances — never stored as fields.
- Single-subsystem commands → instance methods on the subsystem (`this.run(...)`, `this.startEnd(...)`).
- Multi-subsystem commands → methods on `ShooterSystem`/`IntakeSystem`.
- Always `.withName("DescriptiveName")` on composed commands.
- Two stop semantics: `stopHold()` (persistent default, holds zero) vs `stopNow()` (one-shot instant stop).
- Boolean state → `public final Trigger` fields on subsystems (e.g., `atSpeed`, `onTarget`).

### YAMS Motor Framework
- Every subsystem **must** create a `Mechanism` object (even if using `SmartMotorController` directly) — the Mechanism constructor re-applies config.
- `periodic()` **must** call `mechanism.updateTelemetry()`. `simulationPeriodic()` **must** call `mechanism.simIterate()`.
- SparkMax PID units: kP is V/mechanism-rotation, kV is V/(mech rot/s). See `docs/assistant/profile.md` "SparkMax PID/FF unit system".
- `getRotorVelocity()` is **buggy** on SparkWrapper — derive from `mechanismVelocity × gearReduction` instead.
- Use `.clone()` on `SmartMotorControllerConfig` when sharing config across motors with minor differences.

### Constants Organization
- `Constants.java` — runtime flags (`tuningMode`, `RobotType`), drive/PathPlanner gains, safety thresholds.
- `RobotMap.java` — all CAN IDs and DIO channels in one place.
- `ShooterConstants.java` — nested classes per mechanism (`FlywheelConstants`, `TurretConstants`, `HoodConstants`, etc.) + `ShooterModel` two-point RPM model + `HybridAimingConstants`.
- `FieldConstants.java` — field geometry, hub positions, AprilTag layouts. All blue-origin; use `AllianceFlipUtil.apply()` for red.
- Use `import static` for constants to avoid verbose prefixes.

### Logging & Telemetry
- AdvantageKit `Logger.recordOutput("Key", value)` for all telemetry.
- Phoenix signals: cache `StatusSignal<>` references as fields, refresh via `PhoenixUtil.refresh()` in IO `updateInputs()`.
- Use `Constants.CommsConstants.DEFAULT_TELEMETRY_HZ` (50 Hz) for signal update frequency.
- `LoggedTunableNumber` for dashboard-adjustable values; supports `.get()` / `.set()` with NT write-through.

### Shooting Pipeline
- `ShooterModel` — two-point linear RPM interpolation from distance (`ShooterConstants.ShooterModel.dMin`/`dMax`).
- `ShootingLookupTable` — hood angle + TOF lookup by distance. Modes: `HUB` and `PASS`.
- `kRPMFudgeRPM` / `kDistanceFudgeMeters` — operator-adjustable mid-match offsets (±50 RPM via RB/LB, clamped ±200).
- Hybrid aiming: drivetrain handles coarse heading, turret handles residual (clamped to asymmetric travel window `[kTurretMinDeg, kTurretMaxDeg]` = [−90°, +155°]). See `docs/HybridAiming.md`.

### HubShiftUtil — Shift-Gated Feeding
- 2026 game: teleop (140 s) is split into alternating scoring windows (shifts). Only one alliance may score into the hub per shift.
- `HubShiftUtil.initialize()` called once at teleop start. `getShiftedShiftInfo()` returns current shift state + `active` boolean.
- Feed commands in `ShooterSystem.aimAndShoot()` are **shift-gated**: the spindexer/kicker only run when `shiftInfo.active()` is true. The flywheel and turret aim continuously.
- TOF (time-of-flight) fudge factors shift the active window boundaries so the robot proactively starts/stops feeding before the official boundary, accounting for ball travel time. These are derived from `ShootingLookupTable` min/max TOF — do NOT hardcode them in `HubShiftUtil`.
- `aimAndShootPass()` is **ungated** (fires freely regardless of shift state).
- FMS game-specific message determines which alliance starts. Fallback: opposite of own alliance. Manual override via `setAllianceWinOverride()`.

### Build & Deploy
- Build: `.\gradlew compileJava`
- Simulate: `.\gradlew simulateJava`
- Deploy check: `Constants.CheckDeploy` prevents deploying SIMBOT config.
- PR check: `Constants.CheckPullRequest` ensures COMPBOT is selected.
- Spotless formatting: `.\gradlew spotlessApply`
- **Robot reboot required** after SparkMax config changes (flash persistence).

## WPILib Command Composition Quick Reference

| Method | Group Type | Ends When |
|--------|-----------|-----------|
| `alongWith()` | `ParallelCommandGroup` | **ALL** commands finish |
| `raceWith()` | `ParallelRaceGroup` | **ANY** command finishes |
| `deadlineFor()` | `ParallelDeadlineGroup` | The **deadline** (calling) command finishes; interrupts all others |
| `andThen()` | `SequentialCommandGroup` | Commands run in order; ends when last finishes |

### Command Lifecycle Types

| Factory / Class | Behavior |
|----------------|----------|
| `InstantCommand` / `runOnce()` | Runs once, finishes immediately |
| `RunCommand` / `run()` | Runs every cycle, **never finishes on its own** |
| `runEnd()` | Runs every cycle with an end action, **never finishes on its own** |
| `startEnd()` | Runs a start action on init; runs an end action when interrupted, **never finishes on its own** |

## Common Pitfalls
- **PathPlanner `NamedCommands` must terminate** — an infinite command (e.g., `run()`) will stall the entire auto sequence. Use `runOnce()`, `withTimeout()`, or `until()` for event markers.
- **CTRE `StatusSignal` values must be refreshed** before reading; stale signals return old data silently. Always call `PhoenixUtil.refresh()` in IO `updateInputs()`.
- **SparkMax persists config to flash** — after code changes to YAMS config, a **robot reboot** is required. Stale flash params cause unexpected behavior (wrong speed, inversions, etc.).
- **`getRotorVelocity()` is buggy on SparkWrapper** — reads position instead of velocity. Derive motor velocity as `mechanismVelocity × gearReduction`.
- **`Mechanism` object is mandatory** — even if using `SmartMotorController` directly, always create the `Mechanism`. Its constructor re-applies config (soft limits, etc.) to the motor.
- **Command groups block ALL involved subsystems** for their entire duration — default commands do NOT run during a composition. Use `Trigger` for loose coupling when this is too rigid.
- **Don't store command instances as fields** — each use must get a fresh instance from a factory method. Reusing a command causes "command already scheduled" errors or silent no-ops.
- **`Logger.recordOutput` keys must be unique** — duplicate keys silently overwrite each other. Group by subsystem path (e.g., `"Shooter/Flywheel/VelocityRPM"`).

## Naming Conventions

### Classes
- **Subsystems** — PascalCase with `Subsystem` suffix: `FlywheelSubsystem`, `TurretSubsystem`, `IntakePivotSubsystem`.
- **IO interfaces** — `<Subsystem>IO` pattern (no `Subsystem` suffix): `VisionIO`, `GyroIO`, `ModuleIO`.
- **IO implementations** — `<Subsystem>IO<Type>` pattern: `VisionIOPhotonVision`, `VisionIOPhotonVisionSim`, `ModuleIOTalonFX`, `ModuleIOSim`.
- **Systems** (multi-subsystem coordinators) — PascalCase with `System` suffix: `ShooterSystem`, `IntakeSystem`.
- **Utility classes** — descriptive purpose: `AllianceFlipUtil`, `HubShiftUtil`, `PhoenixUtil`.

### Enum Values (States)
- **Present participles** for action states: `INTAKING`, `SHOOTING`, `SPINNING_UP`, `FEEDING`.
- **Adjectives/nouns** for condition states: `IDLE`, `READY`, `AT_SETPOINT`, `DISABLED`.
- Always `ALL_CAPS` with underscores. Spell out words fully (no `2` for `TO`, `4` for `FOR`).

### Commands & Methods
- **Command factory methods** — camelCase, verb-first: `setVelocity()`, `aimAndShoot()`, `feedShooter()`, `stopHold()`.
- **No `Factory` or `Cmd` in method names** — the return type `Command` is sufficient.
- Always `.withName("DescriptiveName")` on composed commands for readable scheduler traces.

### Constants
- `ALL_CAPS` with descriptive names **including units**: `kMaxVelocityRPM`, `kStallCurrentAmps`, `kTimeoutSeconds`.
- Prefix with `k` per WPILib convention: `kTurretMinDeg`, `kRPMFudgeRPM`.

## Noop IO Requirement
Every robot configuration in `RobotContainer` **must** initialize **all** subsystems. For hardware not present on a given config (e.g., SIM has no real cameras), use anonymous Noop IO implementations:
```java
new VisionIO() {}  // all-zero no-op, no hardware calls
```
This eliminates null checks and ensures Systems and bindings work uniformly across all configs (COMPBOT, SIMBOT, etc.). Never leave a subsystem reference as `null`.

## PR Review Guidelines

When reviewing a PR (or when asked to review code), follow these rules:

### Focus
- **Flag breaking changes, not nitpicks.** Things that could break the robot, change runtime behavior unexpectedly, or affect other robot configs.
- **Only review lines in the diff.** Do not flag pre-existing issues in touched files.
- **Evaluate correctness and intent.** Does the code do what the PR says? Does it introduce regressions?

### What Counts as a Breaking Change
- **Behavioral changes**: command lifecycle changes (`runEnd` → `runOnce`), removed stop actions, changed motor inversions or sensor polarity.
- **Cross-config impact**: changes to shared code (Systems, RobotContainer bindings) that affect configs other than the one being modified.
- **CAN ID conflicts**: two constants sharing the same ID on the same bus.
- **Disabled functionality**: commenting out periodic calculations, safety checks, or sensor reads that other code depends on.
- **API signature changes**: renamed or removed public methods that may have callers outside the diff.

### What Is NOT Worth Flagging
- Style, formatting, or naming preferences (Spotless handles formatting).
- Missing Javadoc or comments.
- Code organization or method extraction choices.
- Conventions that don't affect correctness.

## Do / Don't / Ask First

### Do
- Write clear, commented code with `.withName()` on all composed commands.
- Log important values with `Logger.recordOutput()` for debugging.
- Use units consistently (meters, radians, seconds) and include units in constant names.
- Run `.\gradlew compileJava` after changes to verify they compile.
- Run `.\gradlew spotlessApply` before finalizing changes.

### Don't
- **Don't** hand-edit `generated/TunerConstants.java` — it's auto-generated by CTRE Tuner X.
- **Don't** hardcode CAN IDs or DIO ports — use `RobotMap.java`.
- **Don't** leave subsystem references as `null` in any robot config — use Noop IOs.
- **Don't** store command instances as fields or reuse them across bindings.
- **Don't** put multi-subsystem command logic inside a single subsystem class.
- **Don't** delete or modify existing tests without explicit instruction.
- **Don't** use unnamed numeric literals — give values descriptive names with units.

### Ask First
- Before adding or upgrading vendor dependencies.
- Before restructuring folder layout or renaming packages.
- Before changing CAN IDs, motor inversions, or sensor polarity in `RobotMap.java`.
- Before modifying autonomous routines or PathPlanner paths.
- Before changing the architecture (e.g., merging subsystems, adding a new System coordinator).
- Before modifying `Constants.RobotType` or deploy/PR check logic.

## Vendor Dependencies & Javadocs

| Library | Version | Javadoc | User Guide |
|---------|---------|---------|------------|
| WPILib | 2026 | [Javadoc](https://github.wpilib.org/allwpilib/docs/release/java/index.html) | [Docs](https://docs.wpilib.org/en/stable/) |
| CTRE Phoenix 6 | 26.1.3 | [Javadoc](https://api.ctr-electronics.com/phoenix6/stable/java/index.html) | [Docs](https://v6.docs.ctr-electronics.com/en/stable/) |
| REVLib | 2026 | [Javadoc](https://codedocs.revrobotics.com/java/com/revrobotics/package-summary.html) | [Docs](https://docs.revrobotics.com/revlib) |
| PathplannerLib | 2026.1.2 | [Javadoc](https://pathplanner.dev/api/java/) | [Docs](https://pathplanner.dev/home.html) |
| AdvantageKit | 26.0.0 | [Javadoc](https://docs.advantagekit.org/javadoc/) | [Docs](https://docs.advantagekit.org/) |
| PhotonVision | 2026 | [Javadoc](https://photonvision.github.io/photonvision/javadoc/) | [Docs](https://docs.photonvision.org/) |
| YAMS | 2026.1.17 | — | [Docs](https://yagsl.gitbook.io/yams) |

## Key File Map
| Area | Path |
|------|------|
| Robot entry | `src/main/java/frc/robot/Robot.java` |
| Wiring & bindings | `src/main/java/frc/robot/RobotContainer.java` |
| Shooter coordinator | `src/main/java/frc/robot/systems/ShooterSystem.java` |
| Intake coordinator | `src/main/java/frc/robot/systems/IntakeSystem.java` |
| Drive commands | `src/main/java/frc/robot/commands/DriveCommands.java` |
| CAN IDs | `src/main/java/frc/robot/constants/RobotMap.java` |
| Shooter constants | `src/main/java/frc/robot/subsystems/shooter/ShooterConstants.java` |
| Swerve config (generated) | `src/main/java/frc/robot/generated/TunerConstants.java` |
| Field geometry | `src/main/java/frc/robot/constants/FieldConstants.java` |
| Full assistant context | `docs/assistant/profile.md` |
| Tuning reference | `docs/TuningGuide.md`, `docs/TestModeTuning.md` |

## Library Source Access
Use `github_repo` tool (not JAR decompilation) for dependency internals:
- **YAMS**: `Yet-Another-Software-Suite/YAMS`
- **PathPlanner**: `mjansen4857/pathplanner`
- **AdvantageKit**: `Mechanical-Advantage/AdvantageKit`
- **PhotonVision**: `PhotonVision/photonvision`
- **WPILib**: `wpilibsuite/allwpilib`

## Deep Reference
For extended context (YAMS deep reference, Phoenix 6 API patterns, WPILib tuning theory, competition status, and a starter prompt for new AI sessions), read `docs/assistant/profile.md`. For a log of past assistant sessions and design decisions, see `docs/assistant/history.md`. The `docs/assistant/plan-format.md` file defines a structured planner→executor AI workflow template.
