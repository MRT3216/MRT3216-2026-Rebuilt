// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.subsystems.intake.IntakeConstants.Rollers.kTargetAngularVelocity;
import static frc.robot.subsystems.shooter.ShooterConstants.kRefinementConvergenceEpsilon;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
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
import frc.robot.subsystems.lights.LEDSubsystem;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.KickerSubsystem;
import frc.robot.subsystems.shooter.ShooterConstants.FlywheelConstants;
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
import frc.robot.util.TuningDashboard;
import frc.robot.util.shooter.HybridTurretUtil;
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

    // TODO: Uncomment when LEDs are physically wired to the roboRIO PWM port.
    // Verify RobotMap.LEDs.kPort and Constants.LEDsConstants.kNumLEDs match the
    // actual hardware before enabling.
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
     * Current shoot mode — toggled by operator stick presses during teleop. Starts at {@link
     * ShootMode#FULL} (full SOTF). Read by {@code aimAndShoot} each loop cycle so changes take effect
     * immediately, even mid-shot.
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

                    vision =
                            new Vision(
                                    drive::addVisionMeasurement,
                                    new VisionIOPhotonVisionSim(
                                            VisionConstants.cameraFrontName,
                                            VisionConstants.robotToCameraFront,
                                            drive::getPose),
                                    new VisionIOPhotonVisionSim(
                                            VisionConstants.cameraLeftName,
                                            VisionConstants.robotToCameraLeft,
                                            drive::getPose),
                                    new VisionIOPhotonVisionSim(
                                            VisionConstants.cameraRightName,
                                            VisionConstants.robotToCameraRight,
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
        // Use duty-cycle intake for auto until pivot PID/FF gains are tuned.
        // Switch to intakeSystem.intake() / intakeSystem.agitate() once tuned.
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
        NamedCommands.registerCommand("Agitate", intakeSystem.dutyCycleAgitate());
        NamedCommands.registerCommand("Stop Shooter", shooterSystem.stopShooting());

        setupAutoChooser();
        // setupSysid();
        configureDefaultCommands();
        configureButtonBindings();

        // Initialize the tuning Shuffleboard tab only when tuning mode is active.
        // The tab never appears on the Elastic dashboard during competition.
        if (Constants.tuningMode) {
            TuningDashboard.initialize(drive, turretSubsystem, hoodSubsystem, flywheelSubsystem);
        }
    }

    // endregion

    // region Default commands

    private void configureDefaultCommands() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(
                DriveCommands.joystickDrive(
                        drive,
                        () -> -driverController.getLeftY(),
                        () -> -driverController.getLeftX(),
                        () -> -driverController.getRightX()));

        // Kicker should stop (do not coast) when idle — use persistent stopHold()
        // so the subsystem remains at zero output when no one owns it.
        kickerSubsystem.setDefaultCommand(kickerSubsystem.stopHold());
        if (Constants.tuningMode) {
            // D-pad turret rotation for tuning mode.
            // Hold D-pad RIGHT to rotate clockwise, D-pad LEFT to rotate
            // counter-clockwise. Uses the POV hat so it does NOT conflict with
            // LB/RB intake bindings in configureTestButtonBindings().
            // Rate is in degrees per 20 ms loop (~180°/s at full speed).
            final double kRotateRateDegPerLoop = 3.6; // 180 deg/s ÷ 50 Hz
            final double kSoftMin = -190.0;
            final double kSoftMax = 190.0;
            // Re-seeded to the turret's current position each time the command initializes
            // (i.e., on every enable) so the turret doesn't snap to a stale setpoint.
            final double[] turretAccumulator = {0.0};

            turretSubsystem.setDefaultCommand(
                    Commands.sequence(
                                    Commands.runOnce(
                                            () -> turretAccumulator[0] = turretSubsystem.getPosition().in(Degrees)),
                                    turretSubsystem.setAngle(
                                            () -> {
                                                int pov = driverController.getHID().getPOV();
                                                boolean cw = (pov == 90); // D-pad RIGHT
                                                boolean ccw = (pov == 270); // D-pad LEFT
                                                if (cw && !ccw) {
                                                    turretAccumulator[0] -= kRotateRateDegPerLoop;
                                                } else if (ccw && !cw) {
                                                    turretAccumulator[0] += kRotateRateDegPerLoop;
                                                }
                                                // Clamp to soft limits
                                                turretAccumulator[0] =
                                                        Math.max(kSoftMin, Math.min(kSoftMax, turretAccumulator[0]));
                                                return Degrees.of(turretAccumulator[0]);
                                            }))
                            .withName("Turret_TuningDpadControl"));

            // In tuning mode the flywheel should stay idle — no hub-shift pre-spin.
            // HubShiftUtil returns unpredictable state in sim (no FMS data), which
            // would cause the flywheel to spin at seemingly random times.
            flywheelSubsystem.setDefaultCommand(flywheelSubsystem.stopHold());
        } else {
            // Shift-aware turret tracking default command.
            //
            // When the hub shift is active the turret continuously tracks the alliance
            // hub center (HUB table). When the shift is inactive it tracks the nearest
            // pass target landing zone (PASS table). Motion-compensated via
            // HybridTurretUtil.computeMovingShot() each periodic loop.
            //
            // This default is preempted by aimAndShoot / aimAndShootPass when the
            // driver holds a trigger — the subsystem requirement ensures the trigger
            // command takes priority, and tracking resumes automatically on release.

            // Build the two lookup tables once — reused every loop by the turret default.
            var hubTable = new ShootingLookupTable(ShootingLookupTable.Mode.HUB);
            var passTable = new ShootingLookupTable(ShootingLookupTable.Mode.PASS);

            turretSubsystem.setDefaultCommand(
                    turretSubsystem
                            .setAngle(
                                    () -> {
                                        var shift = HubShiftUtil.getShiftedShiftInfo();
                                        var pose = drive.getPose();
                                        var speeds = drive.getChassisSpeeds();

                                        Translation3d target;
                                        ShootingLookupTable table;
                                        if (shift.active()) {
                                            target = AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint);
                                            table = hubTable;
                                        } else {
                                            // Flip targets to current alliance BEFORE comparing Y
                                            // so the nearest-target pick works correctly on both
                                            // alliances.
                                            var left = AllianceFlipUtil.apply(FieldConstants.PassTarget.left);
                                            var right = AllianceFlipUtil.apply(FieldConstants.PassTarget.right);
                                            double robotY = pose.getY();
                                            target =
                                                    Math.abs(robotY - left.getY()) < Math.abs(robotY - right.getY())
                                                            ? left
                                                            : right;
                                            table = passTable;
                                        }
                                        return HybridTurretUtil.computeMovingShot(
                                                        pose, speeds, target, 3, kRefinementConvergenceEpsilon, table)
                                                .turretAzimuth();
                                    })
                            .withName("Turret_DefaultTracking"));

            // Hood returns to 0° when not actively shooting — prevents decapitation
            // under the trench. Hood tracking only happens inside aimAndShoot /
            // aimAndShootPass while the driver holds a trigger.
            hoodSubsystem.setDefaultCommand(
                    hoodSubsystem.setAngle(Degrees.of(0)).withName("Hood_DefaultStow"));

            // Flywheel pre-spins automatically when the shifted shift is active or
            // within 5 seconds of becoming active — no button required. When the
            // operator holds the right trigger, aimAndShoot preempts this default
            // and tracks the exact speed from the lookup table.
            flywheelSubsystem.setDefaultCommand(
                    flywheelSubsystem
                            .setVelocity(
                                    () -> {
                                        var shift = HubShiftUtil.getShiftedShiftInfo();
                                        if (shift.active() || shift.remainingTime() < 5.0) {
                                            return FlywheelConstants.kFlywheelDefaultVelocity;
                                        }
                                        return edu.wpi.first.units.Units.RPM.of(0);
                                    })
                            .withName("Flywheel_DefaultPreSpin"));
        }

        // Let spindexer coast by default. Use the persistent stopHold() default
        // which disables closed-loop control and keeps the duty/voltage at zero
        // while scheduled (the motor idle mode is COAST so it will freewheel).
        spindexerSubsystem.setDefaultCommand(spindexerSubsystem.stopHold());

        // Ensure intake rollers default to stopped when no command is running —
        // they should not coast, so use the persistent stopHold() default.
        intakeRollersSubsystem.setDefaultCommand(intakeRollersSubsystem.stopHold());

        // Ensure intake pivot holds its commanded setpoint when no one owns it so
        // live tuning and dashboard writes persist.
        intakePivotSubsystem.setDefaultCommand(
                // intakePivotSubsystem.setAngle(() -> intakePivotSubsystem.getPosition()));
                intakePivotSubsystem.set(0));
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
     * <p><b>Driver</b> owns driving and intake. <b>Operator</b> owns shooting, shoot-mode selection,
     * and secondary ball-handling overrides. Longer-lived or tuning helpers belong in {@link
     * #configureTestButtonBindings()} and should be enabled only in tuning mode.
     */
    private void configureRealButtonBindings() {
        // ── Driver: intake ──────────────────────────────────────────────

        // Right trigger: duty-cycle intake (deploy arm via timed pulse, then run
        // rollers). Switch to intakeSystem.intake() once pivot PID/FF gains are tuned.
        driverController.rightTrigger().whileTrue(intakeSystem.dutyCycleIntake());
        // TODO: Wire intaking LED when intake is running:
        // driverController
        //         .rightTrigger()
        //         .onTrue(ledSubsystem.setIntakingLEDCommand(() -> true))
        //         .onFalse(ledSubsystem.setIntakingLEDCommand(() -> false));

        // Left trigger immediately stops rollers.
        driverController.leftTrigger().onTrue(intakeSystem.stopRollers());

        // Right bumper: duty-cycle agitate while held, then deploy on release.
        // Switch to intakeSystem.agitate() / intakeSystem.deploy() once tuned.
        driverController
                .rightBumper()
                .whileTrue(intakeSystem.dutyCycleAgitate())
                .onFalse(intakeSystem.dutyCycleDeploy());

        // Left bumper: eject balls from intake rollers while held.
        driverController.leftBumper().whileTrue(intakeRollersSubsystem.ejectBalls());

        // ── Operator: shooting ──────────────────────────────────────────

        // Right trigger: hold to aim + feed a hub shot (shift-gated).
        operatorController
                .rightTrigger()
                .whileTrue(
                        shooterSystem.aimAndShoot(
                                () -> drive.getPose(),
                                () -> drive.getChassisSpeeds(),
                                () -> AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint),
                                3,
                                ShootingLookupTable.Mode.HUB,
                                () -> currentShootMode));
        // TODO: Wire aim-lock LED when shooting is active:
        // operatorController
        //         .rightTrigger()
        //         .onTrue(ledSubsystem.setAimLockLEDCommand(() -> true))
        //         .onFalse(ledSubsystem.setAimLockLEDCommand(() -> false));

        // Left trigger: hold to aim + feed a pass shot. Not shift-gated —
        // feeds freely while held regardless of hub shift state. Turret and hood
        // track the nearest pass target landing zone for the duration.
        operatorController
                .leftTrigger()
                .whileTrue(
                        shooterSystem.aimAndShootPass(
                                () -> drive.getPose(), () -> drive.getChassisSpeeds(), 3));
        // TODO: Wire aim-lock LED when pass shooting is active:
        // operatorController
        //         .leftTrigger()
        //         .onTrue(ledSubsystem.setAimLockLEDCommand(() -> true))
        //         .onFalse(ledSubsystem.setAimLockLEDCommand(() -> false));

        // ── Operator: secondary ball-handling & overrides ───────────────

        // Right bumper: manually run intake rollers inward (override).
        operatorController.rightBumper().whileTrue(intakeRollersSubsystem.intakeBalls());

        // Left bumper: clear / unjam shooter system while held.
        operatorController.leftBumper().whileTrue(shooterSystem.clearShooterSystem());

        // ── Operator: shoot-mode toggles ────────────────────────────────

        // Left stick press: toggle STATIC_DISTANCE mode
        operatorController
                .leftStick()
                .onTrue(
                        Commands.runOnce(
                                () -> {
                                    if (currentShootMode == ShootMode.STATIC_DISTANCE) {
                                        currentShootMode = ShootMode.FULL;
                                    } else {
                                        currentShootMode = ShootMode.STATIC_DISTANCE;
                                    }
                                    Logger.recordOutput("ShooterTelemetry/shootMode", currentShootMode.name());
                                }));

        // Right stick press: toggle FULL_STATIC mode (battle-tested comp fallback)
        operatorController
                .rightStick()
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

        // ── Shift-end rumble ────────────────────────────────────────────
        // Pulse rumble once per second in the last 5s of an active shift —
        // mirrors 6328's end-of-shift warning. Both controllers rumble so both
        // driver and operator have situational awareness of shift boundaries.
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
     * Enable test/tuning-specific bindings. These are small helpers intended for development and
     * should not be active during normal competition operation. This method should be idempotent in
     * higher-level flows (constructor only currently). Add more bindings here as needed when
     * experimenting.
     */
    private void configureTestButtonBindings() {
        driverController
                .a()
                .whileTrue(
                        shooterSystem.aim(
                                () -> drive.getPose(),
                                () -> drive.getChassisSpeeds(),
                                () -> AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint),
                                3,
                                ShootingLookupTable.Mode.HUB));

        driverController.b().whileTrue(intakeRollersSubsystem.setVelocity(kTargetAngularVelocity));
        driverController.x().whileTrue(spindexerSubsystem.feedShooter());
        driverController.y().whileTrue(kickerSubsystem.feedShooter());

        // Duty-cycle intake: deploy arm via timed pulse, then run rollers while held.
        // Switch to intakeSystem.intake() once pivot PID/FF gains are tuned.
        driverController.rightBumper().whileTrue(intakeSystem.dutyCycleIntake());

        // Left bumper immediately stops rollers.
        driverController.leftBumper().onTrue(intakeSystem.stopRollers());

        // Left trigger: duty-cycle agitate while held, re-deploy on release.
        // Switch to intakeSystem.agitate() once pivot PID/FF gains are tuned.
        driverController
                .leftTrigger()
                .whileTrue(intakeSystem.dutyCycleAgitate())
                .onFalse(intakeSystem.dutyCycleDeploy());

        driverController.rightTrigger().whileTrue(shooterSystem.testShoot(() -> drive.getPose()));
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
     * SysId helper: registers SysId routines on the dashboard.
     *
     * <p>Disabled by default. To enable SysId options, call {@code setupSysid()} from the constructor
     * and uncomment the implementation inside this method.
     */
    @SuppressWarnings("unused")
    private void setupSysid() {
        // Set up SysId routines
        autoChooser = new LoggedDashboardChooser<>("Tuning", AutoBuilder.buildAutoChooser());
        autoChooser.addOption(
                "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
        autoChooser.addOption(
                "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)",
                drive.sysIdQuasistatic(
                        edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)",
                drive.sysIdQuasistatic(
                        edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
                "Drive SysId (Dynamic Forward)",
                drive.sysIdDynamic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Dynamic Reverse)",
                drive.sysIdDynamic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse));
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
