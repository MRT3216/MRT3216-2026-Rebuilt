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
- Hybrid aiming: drivetrain handles coarse heading, turret handles residual (clamped to asymmetric travel window `[kTurretMinDeg, kTurretMaxDeg]` = [−90°, +130°]). See `docs/HybridAiming.md`.

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
