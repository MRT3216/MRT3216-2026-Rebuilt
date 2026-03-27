// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.subsystems.shooter.ShooterConstants.kRPMFudgeRPM;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollersSubsystem;
import frc.robot.subsystems.lights.*;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.KickerSubsystem;
import frc.robot.subsystems.shooter.ShooterConstants.ShootMode;
import frc.robot.subsystems.shooter.SpindexerSubsystem;
import frc.robot.subsystems.shooter.TurretSubsystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.systems.IntakeSystem;
import frc.robot.systems.ShooterSystem;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.HubShiftUtil;
import frc.robot.util.RobotMapValidator;
import frc.robot.util.shooter.ShootingLookupTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 *
 * <p>Container for robot subsystems, commands and button bindings.
 */
public class RobotContainer {
    // region Subsystems & systems

    // Subsystems
    private final Drive drive;
    private final Vision vision;
    private final FlywheelSubsystem flywheelSubsystem = new FlywheelSubsystem();
    private final KickerSubsystem kickerSubsystem = new KickerSubsystem();
    private final TurretSubsystem turretSubsystem = new TurretSubsystem();
    private final SpindexerSubsystem spindexerSubsystem = new SpindexerSubsystem();
    private final HoodSubsystem hoodSubsystem = new HoodSubsystem();
    private final IntakePivotSubsystem intakePivotSubsystem = new IntakePivotSubsystem();
    private final IntakeRollersSubsystem intakeRollersSubsystem = new IntakeRollersSubsystem();
    private final LEDSubsystem ledSubsystem = new LEDSubsystem();

    // Aggregated shooter system
    private final ShooterSystem shooterSystem =
            new ShooterSystem(
                    flywheelSubsystem, kickerSubsystem, spindexerSubsystem, turretSubsystem, hoodSubsystem);

    private final IntakeSystem intakeSystem =
            new IntakeSystem(intakeRollersSubsystem, intakePivotSubsystem);

    // Controller
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    /**
     * Current shoot mode — toggled by the Y button during teleop. Starts at {@link ShootMode#FULL}
     * (full SOTF). Pressing Y toggles to {@link ShootMode#FULL_STATIC} (turret locked at 0°, no lead
     * compensation) and back. Read by {@code aimAndShoot} each loop cycle so changes take effect
     * immediately, even mid-shot. take effect immediately, even mid-shot.
     */
    private ShootMode currentShootMode = ShootMode.FULL;

    // Dashboard inputs (initialized by setupAutoChooser when enabled)
    private LoggedDashboardChooser<Command> autoChooser;

    // endregion

    // region Constructor

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Validate RobotMap wiring early at startup and warn if duplicate IDs are
        // found.
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
        // Force VisionConstants class-load (parses AprilTag JSON) at startup so the
        // first Vision.periodic() call does not eat the ~250-500ms file I/O cost.
        frc.robot.subsystems.vision.VisionConstants.aprilTagLayout.getFieldLength();
        RobotMapValidator.validate();

        switch (Constants.getMode()) {
            case REAL:
                {
                    // Real robot, instantiate hardware IO implementations
                    drive =
                            new Drive(
                                    new GyroIOPigeon2(),
                                    new ModuleIOTalonFX(TunerConstants.FrontLeft),
                                    new ModuleIOTalonFX(TunerConstants.FrontRight),
                                    new ModuleIOTalonFX(TunerConstants.BackLeft),
                                    new ModuleIOTalonFX(TunerConstants.BackRight));

                    vision =
                            new Vision(
                                    drive::addVisionMeasurement,
                                    new VisionIOPhotonVision(
                                            VisionConstants.cameraFrontName, VisionConstants.robotToCameraFront),
                                    new VisionIOPhotonVision(
                                            VisionConstants.cameraLeftName, VisionConstants.robotToCameraLeft),
                                    new VisionIOPhotonVision(
                                            VisionConstants.cameraRightName, VisionConstants.robotToCameraRight),
                                    new VisionIOPhotonVision(
                                            VisionConstants.cameraBackName, VisionConstants.robotToCameraBack));

                    break;
                }
            case SIM:
                {
                    // Sim robot, instantiate physics sim IO implementations
                    drive =
                            new Drive(
                                    new GyroIO() {},
                                    new ModuleIOSim(TunerConstants.FrontLeft),
                                    new ModuleIOSim(TunerConstants.FrontRight),
                                    new ModuleIOSim(TunerConstants.BackLeft),
                                    new ModuleIOSim(TunerConstants.BackRight));

                    // Use only front + back cameras in sim to keep the loop under
                    // 20ms. PhotonVision's VisionSystemSim is CPU-heavy (~8-10ms per
                    // camera); 4 cameras blow the budget. Two cameras still give full
                    // 360° pose-estimation coverage for testing.
                    vision =
                            new Vision(
                                    drive::addVisionMeasurement,
                                    new VisionIOPhotonVisionSim(
                                            VisionConstants.cameraFrontName,
                                            VisionConstants.robotToCameraFront,
                                            drive::getPose),
                                    new VisionIOPhotonVisionSim(
                                            VisionConstants.cameraBackName,
                                            VisionConstants.robotToCameraBack,
                                            drive::getPose));

                    break;
                }

            default:
                {
                    // Replayed robot, disable IO implementations
                    drive =
                            new Drive(
                                    new GyroIO() {},
                                    new ModuleIO() {},
                                    new ModuleIO() {},
                                    new ModuleIO() {},
                                    new ModuleIO() {});
                    // (Use same number of dummy implementations as the real robot)
                    vision =
                            new Vision(
                                    drive::addVisionMeasurement,
                                    new VisionIO() {},
                                    new VisionIO() {},
                                    new VisionIO() {},
                                    new VisionIO() {});
                    break;
                }
        }

        // Register auto commands
        NamedCommands.registerCommand("Run Intake", intakeSystem.dutyCycleIntake());
        NamedCommands.registerCommand(
                "Aim and Shoot",
                shooterSystem.aimAndShoot(
                        () -> drive.getPose(),
                        () -> drive.getChassisSpeeds(),
                        () -> AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint),
                        3,
                        ShootingLookupTable.Mode.HUB,
                        () -> currentShootMode));
        NamedCommands.registerCommand(
                "Hybrid Aim and Shoot",
                shooterSystem.hybridAimAndShoot(
                        () -> drive.getPose(),
                        () -> drive.getChassisSpeeds(),
                        () -> AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint),
                        3,
                        ShootingLookupTable.Mode.HUB,
                        () -> currentShootMode));
        NamedCommands.registerCommand("Agitate", intakeSystem.dutyCycleAgitate());
        NamedCommands.registerCommand("Stop Shooter", shooterSystem.stopShooting());

        setupAutoChooser();
        configureDefaultCommands();
        configureButtonBindings();
    }

    // endregion

    // region Default commands

    private void configureDefaultCommands() {
        // Hybrid drive default: drivetrain auto-rotates toward the hub when
        // the driver holds right trigger (aimEnabled). Full manual control
        // otherwise. See docs/HybridAiming.md.
        drive.setDefaultCommand(
                DriveCommands.joystickDriveAimAtTarget(
                        drive,
                        () -> -driverController.getLeftY(),
                        () -> -driverController.getLeftX(),
                        () -> -driverController.getRightX(),
                        () -> AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint).toTranslation2d(),
                        () -> drive.getPose(),
                        () -> driverController.getRightTriggerAxis() > 0.5));

        // Kicker should stop (do not coast) when idle — use persistent stopHold()
        // so the subsystem remains at zero output when no one owns it.
        kickerSubsystem.setDefaultCommand(kickerSubsystem.stopHold());

        // Turret holds at home (0°) when not actively shooting. The hybrid
        // aim-and-shoot commands (RT / LT) take over while the driver holds a
        // trigger — subsystem requirement preempts this default automatically.
        turretSubsystem.setDefaultCommand(
                turretSubsystem.setAngle(Degrees.of(0)).withName("Turret_DefaultStow"));

        // Hood returns to 0° when not actively shooting — prevents decapitation
        // under the trench. Hood tracking only happens inside aimAndShoot /
        // aimAndShootPass while the driver holds a trigger.
        hoodSubsystem.setDefaultCommand(
                hoodSubsystem.setAngle(Degrees.of(0)).withName("Hood_DefaultStow"));

        // Flywheel stays idle by default — it only spins when the driver
        // holds right trigger (hybridAimAndShoot commands exact LUT speed).
        flywheelSubsystem.setDefaultCommand(flywheelSubsystem.stopHold());

        // Let spindexer coast by default. Use the persistent stopHold() default
        // which disables closed-loop control and keeps the duty/voltage at zero
        // while scheduled (the motor idle mode is COAST so it will freewheel).
        spindexerSubsystem.setDefaultCommand(spindexerSubsystem.stopHold());

        // Ensure intake rollers default to stopped when no command is running —
        // they should not coast, so use the persistent stopHold() default.
        intakeRollersSubsystem.setDefaultCommand(intakeRollersSubsystem.stopHold());

        // Ensure intake pivot holds its commanded setpoint when no one owns it so
        // live tuning and dashboard writes persist.
        intakePivotSubsystem.setDefaultCommand(intakePivotSubsystem.set(0));
    }

    // endregion

    // region Button bindings

    private void configureButtonBindings() {
        if (Constants.tuningMode) {
            configureTestButtonBindings();
        } else {
            configureRealButtonBindings();
        }

        // Reset gyro to 0° when the Start button is pressed (available in both REAL and
        // SIM). Use the controller "start()" binding here intentionally — if you prefer
        // Back change the binding to driverController.back().
        driverController.start().onTrue(resetGyroZeroCommand());

        // Warn both controllers with continuous rumble if FMS has not sent game-specific
        // data (hub winner) within 1 second of teleop start and no manual override is set.
        // Only active when connected to a real FMS — otherwise bench testing would
        // trigger permanent rumble since the game-specific message is always empty.
        // Mirrors 6328's autoWinnerNotSet alert.
        Timer teleopElapsedTimer = new Timer();
        RobotModeTriggers.teleop().onTrue(Commands.runOnce(teleopElapsedTimer::restart));
        RobotModeTriggers.teleop()
                .and(DriverStation::isFMSAttached)
                .and(() -> DriverStation.getGameSpecificMessage().isEmpty())
                .and(() -> HubShiftUtil.getAllianceWinOverride().isEmpty())
                .and(() -> teleopElapsedTimer.hasElapsed(1.0))
                .whileTrue(
                        Commands.runEnd(
                                () -> {
                                    driverController.setRumble(RumbleType.kBothRumble, 1.0);
                                    operatorController.setRumble(RumbleType.kBothRumble, 1.0);
                                },
                                () -> {
                                    driverController.setRumble(RumbleType.kBothRumble, 0.0);
                                    operatorController.setRumble(RumbleType.kBothRumble, 0.0);
                                }));
    }

    /**
     * Configure bindings used on the REAL robot for competition-style operation.
     *
     * <p><b>Driver</b> owns driving and shooting (hub + pass). <b>Operator</b> owns intake,
     * shoot-mode selection, and secondary ball-handling overrides. Longer-lived or tuning helpers
     * belong in {@link #configureTestButtonBindings()} and should be enabled only in tuning mode.
     */
    private void configureRealButtonBindings() {
        // ── Driver: shooting ─────────────────────────────────────────

        // Right trigger: hybrid aim and shoot — turret clamped to asymmetric
        // travel limits (see HybridAimingConstants), drivetrain heading assist
        // (via drive default command), full feed. See docs/HybridAiming.md.
        driverController
                .rightTrigger()
                .whileTrue(
                        shooterSystem.hybridAimAndShoot(
                                () -> drive.getPose(),
                                () -> drive.getChassisSpeeds(),
                                () -> AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint),
                                3,
                                ShootingLookupTable.Mode.HUB,
                                () -> currentShootMode));
        // Aim-lock LED while hub shooting is active.
        driverController
                .rightTrigger()
                .onTrue(ledSubsystem.setAimLockLEDCommand(() -> true))
                .onFalse(ledSubsystem.setAimLockLEDCommand(() -> false));

        // Left trigger: hybrid pass shot — aims at nearest pass target landing
        // zone with turret clamped to travel limits, drivetrain heading assist.
        // Uses PASS lookup table. Feed is ungated (fires freely).
        driverController
                .leftTrigger()
                .whileTrue(
                        shooterSystem.hybridAimAndShootPass(
                                () -> drive.getPose(), () -> drive.getChassisSpeeds(), 3));
        // Aim-lock LED while pass shooting is active.
        driverController
                .leftTrigger()
                .onTrue(ledSubsystem.setAimLockLEDCommand(() -> true))
                .onFalse(ledSubsystem.setAimLockLEDCommand(() -> false));

        // ── Operator: intake ───────────────────────────────────────────

        // Right trigger: hold to intake (deploy arm + run rollers).
        operatorController.rightTrigger().whileTrue(intakeSystem.dutyCycleIntake());

        // Left trigger: hold to reverse intake (eject balls).
        operatorController.leftTrigger().whileTrue(intakeSystem.dutyCycleEject());

        // ── Operator: secondary ball-handling & overrides ───────────────

        // A button: agitate (re-deploy arm + jog rollers) to dislodge stuck balls.
        // On release, only resume intaking if the operator is still holding the
        // right trigger — otherwise the rollers would run indefinitely because
        // dutyCycleIntake() is a RunCommand with no natural end condition.
        operatorController
                .a()
                .whileTrue(intakeSystem.dutyCycleAgitate())
                .onFalse(
                        Commands.either(
                                intakeSystem.dutyCycleIntake(),
                                Commands.none(),
                                () -> operatorController.getRightTriggerAxis() > 0.5));

        // B button: clear / unjam shooter system while held.
        operatorController.b().whileTrue(shooterSystem.clearShooterSystem());

        // X button: defence mode (red/blue strobe LEDs while held).
        operatorController
                .x()
                .onTrue(ledSubsystem.setDefenceModeLEDCommand(() -> true))
                .onFalse(ledSubsystem.setDefenceModeLEDCommand(() -> false));

        // ── Operator: shoot-mode toggle ─────────────────────────────────

        // Y button: toggle between FULL (auto-aim + SOTF) and FULL_STATIC
        // (turret locked at 0°, no lead compensation — manual aim fallback).
        operatorController
                .y()
                .onTrue(
                        Commands.runOnce(
                                () -> {
                                    if (currentShootMode == ShootMode.FULL_STATIC) {
                                        currentShootMode = ShootMode.FULL;
                                    } else {
                                        currentShootMode = ShootMode.FULL_STATIC;
                                    }
                                    Logger.recordOutput("ShooterTelemetry/shootMode", currentShootMode.name());
                                }));

        // ── Operator: RPM fudge adjustment ──────────────────────────────
        // RB/LB adjust the RPM fudge factor in ±50 RPM increments.
        // Writes through to the LoggedTunableNumber so the dashboard
        // Number Slider updates in real time. Clamped to ±200 RPM.

        operatorController
                .rightBumper()
                .onTrue(
                        Commands.runOnce(
                                () -> {
                                    double next = Math.min(kRPMFudgeRPM.get() + 50, 200);
                                    kRPMFudgeRPM.set(next);
                                    Logger.recordOutput("ShooterTelemetry/rpmFudgeRPM", next);
                                }));

        operatorController
                .leftBumper()
                .onTrue(
                        Commands.runOnce(
                                () -> {
                                    double next = Math.max(kRPMFudgeRPM.get() - 50, -200);
                                    kRPMFudgeRPM.set(next);
                                    Logger.recordOutput("ShooterTelemetry/rpmFudgeRPM", next);
                                }));

        // ── Shift-end rumble ────────────────────────────────────────────
        // Pulse rumble once per second in the last 5s of each hub shift —
        // mirrors 6328's end-of-shift warning. remainingTime() counts down
        // within the current shift window (25-30s each), so each pulse fires
        // once per shift boundary. Both controllers rumble so driver and
        // operator have situational awareness of shift transitions.
        for (int i = 1; i <= 5; i++) {
            final double seconds = i;
            new Trigger(() -> HubShiftUtil.getShiftedShiftInfo().remainingTime() < seconds)
                    .and(RobotModeTriggers.teleop())
                    .onTrue(
                            Commands.runEnd(
                                            () -> {
                                                driverController.setRumble(RumbleType.kRightRumble, 1.0);
                                                operatorController.setRumble(RumbleType.kRightRumble, 1.0);
                                            },
                                            () -> {
                                                driverController.setRumble(RumbleType.kRightRumble, 0.0);
                                                operatorController.setRumble(RumbleType.kRightRumble, 0.0);
                                            })
                                    .withTimeout(0.25));
        }
    }

    /**
     * Tuning-mode bindings use <b>only the driver controller</b> so a single person can test in the
     * pit. Triggers use <b>aim only</b> (no flywheel/feed) so balls aren't accidentally fired.
     *
     * <p><b>Driver:</b> RT = hybrid aim hub (no feed), LT = intake, D-pad Down = turret snap to max
     * travel (130°), D-pad Up/Left/Right/diagonals = turret snap angles, A = agitate, B = clear
     * shooter, X = test shoot, Y = toggle shoot mode, RB = +50 RPM fudge, LB = −50 RPM fudge. LED
     * bindings are disabled in tuning mode to save loop time.
     */
    private void configureTestButtonBindings() {
        // ── Driver: aiming ──────────────────────────────────────────────

        // Right trigger: hybrid aim at hub (turret clamped to asymmetric
        // travel limits, drivetrain heading assist). Aim only — no flywheel or feed.
        driverController
                .rightTrigger()
                .whileTrue(
                        shooterSystem.hybridAim(
                                () -> drive.getPose(),
                                () -> drive.getChassisSpeeds(),
                                () -> AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint),
                                3,
                                ShootingLookupTable.Mode.HUB,
                                () -> currentShootMode));
        // LED bindings are skipped in tuning mode to save loop time.

        // ── Driver: intake ──────────────────────────────────────────────

        // Left trigger: hold to intake (deploy arm + run rollers).
        driverController.leftTrigger().whileTrue(intakeSystem.dutyCycleIntake());

        // D-pad down: snap turret to max travel limit (130°) for verifying
        // asymmetric range. Releases fall back to Turret_DefaultStow (0°).
        driverController.povDown().whileTrue(turretSubsystem.setAngle(Degrees.of(130)));

        // ── Driver: turret snap angles ──────────────────────────────────
        // D-pad directions snap the turret to fixed angles while held.
        // Releases fall back to the Turret_DefaultStow (0°) default command.
        driverController.povUp().whileTrue(turretSubsystem.setAngle(Degrees.of(0)));
        driverController.povLeft().whileTrue(turretSubsystem.setAngle(Degrees.of(90)));
        driverController.povRight().whileTrue(turretSubsystem.setAngle(Degrees.of(-90)));
        driverController.povUpLeft().whileTrue(turretSubsystem.setAngle(Degrees.of(45)));
        driverController.povUpRight().whileTrue(turretSubsystem.setAngle(Degrees.of(-45)));

        // ── Driver: ball-handling & shooter overrides ────────────────────

        // A button: agitate (re-deploy arm + jog rollers) to dislodge stuck balls.
        // On release, reset state and redeploy so intake resumes immediately.
        driverController
                .a()
                .whileTrue(intakeSystem.dutyCycleAgitate())
                .onFalse(intakeSystem.dutyCycleIntake());

        // B button: clear / unjam shooter system while held.
        driverController.b().whileTrue(shooterSystem.clearShooterSystem());

        // X button: test shoot (turret at 0°, spin flywheel + feed) while held.
        // Fires without vision — verifies shooter mechanism in the pit.
        driverController.x().whileTrue(shooterSystem.testShoot(() -> drive.getPose()));

        // ── Operator: intake ───────────────────────────────────────────

        // Right trigger: hold to intake (deploy arm + run rollers).
        operatorController.rightTrigger().whileTrue(intakeSystem.dutyCycleIntake());

        // Left trigger: hold to reverse intake (eject balls).
        operatorController.leftTrigger().whileTrue(intakeSystem.dutyCycleEject());

        // ── Driver: shoot-mode toggle ───────────────────────────────────

        // Y button: toggle between FULL and FULL_STATIC.
        driverController
                .y()
                .onTrue(
                        Commands.runOnce(
                                () -> {
                                    if (currentShootMode == ShootMode.FULL_STATIC) {
                                        currentShootMode = ShootMode.FULL;
                                    } else {
                                        currentShootMode = ShootMode.FULL_STATIC;
                                    }
                                    Logger.recordOutput("ShooterTelemetry/shootMode", currentShootMode.name());
                                }));

        // ── Driver: RPM fudge adjustment ────────────────────────────────

        driverController
                .rightBumper()
                .onTrue(
                        Commands.runOnce(
                                () -> {
                                    double next = Math.min(kRPMFudgeRPM.get() + 50, 200);
                                    kRPMFudgeRPM.set(next);
                                    Logger.recordOutput("ShooterTelemetry/rpmFudgeRPM", next);
                                }));

        driverController
                .leftBumper()
                .onTrue(
                        Commands.runOnce(
                                () -> {
                                    double next = Math.max(kRPMFudgeRPM.get() - 50, -200);
                                    kRPMFudgeRPM.set(next);
                                    Logger.recordOutput("ShooterTelemetry/rpmFudgeRPM", next);
                                }));
    }

    // endregion

    // region Private helpers

    // Centralized reset-gyro command so multiple bindings can reuse the same
    // behavior.
    private Command resetGyroZeroCommand() {
        // Reset the drivetrain pose to a zeroed rotation while preserving translation.
        // We explicitly allow this action while disabled so developers can zero the
        // heading from the Driver Station without enabling the robot. This is a local
        // convenience and not a safety-sensitive action (no motors are commanded here).
        return Commands.runOnce(
                        () ->
                                drive.setPose(
                                        AllianceFlipUtil.apply(
                                                new Pose2d(drive.getPose().getTranslation(), new Rotation2d()))),
                        drive)
                .ignoringDisable(true);
    }

    /**
     * AutoChooser helper: creates the dashboard auto chooser and populates it with available
     * autonomous routines.
     */
    private void setupAutoChooser() {
        // "Auto Chooser" matches the topic key the Elastic dashboard subscribes to.
        autoChooser =
                new LoggedDashboardChooser<>(
                        "Auto Chooser", AutoBuilder.buildAutoChooser("Left Bum Rush (No SOTF)"));
        autoChooser.addOption(
                "Left Bum Rush (No SOTF)", new PathPlannerAuto("Left Bum Rush (No SOTF)"));
    }

    // endregion

    // region Autonomous

    /** Returns the command selected on the dashboard to run during autonomous. */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    // endregion
}
