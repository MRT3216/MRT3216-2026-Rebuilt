// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.constants.IntakeConstants.Rollers.kTargetAngularVelocity;
import static frc.robot.constants.ShooterConstants.kRefinementConvergenceEpsilon;

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
import frc.robot.constants.ShooterConstants.FlywheelConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollersSubsystem;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.KickerSubsystem;
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
import frc.robot.util.shooter.HybridTurretUtil;
import frc.robot.util.shooter.ShootingLookupTable;
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

    // Aggregated shooter system
    private final ShooterSystem shooterSystem =
            new ShooterSystem(
                    flywheelSubsystem, kickerSubsystem, spindexerSubsystem, turretSubsystem, hoodSubsystem);

    private final IntakeSystem intakeSystem =
            new IntakeSystem(intakeRollersSubsystem, intakePivotSubsystem);

    // Controller
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    // Dashboard inputs (initialized by setupAutoChooser when enabled)
    private LoggedDashboardChooser<Command> autoChooser;

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
        NamedCommands.registerCommand("Run Intake", intakeSystem.intake());
        NamedCommands.registerCommand(
                "Aim and Shoot",
                shooterSystem.aimAndShoot(
                        () -> drive.getPose(),
                        () -> drive.getChassisSpeeds(),
                        () -> AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint),
                        3,
                        ShootingLookupTable.Mode.HUB));
        NamedCommands.registerCommand("Agitate", intakeSystem.agitate());
        NamedCommands.registerCommand("Stop Shooter", shooterSystem.stopShooting());

        setupAutoChooser();
        // setupSysid();
        configureDefaultCommands();
        configureButtonBindings();
    }

    public void configureDefaultCommands() {
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
            turretSubsystem.setDefaultCommand(
                    turretSubsystem.setAngle(
                            () -> {
                                // Map right-stick to turret angle in degrees (-180..180).
                                // Raw stick: X right = +1, Y up = -1 (standard Xbox convention).
                                // We negate Y so that stick-forward (−Y) = positive turret angle
                                // (forward/away from driver). atan2(y, x) gives standard math angle
                                // (CCW positive, 0° = right). Adjust sign/offset here if turret
                                // direction doesn't match expectation on hardware.
                                double x = driverController.getRightX();
                                double y = -driverController.getRightY(); // invert so forward = positive
                                double deadband = 0.1;
                                if (Math.hypot(x, y) < deadband) {
                                    return turretSubsystem.getTarget();
                                }
                                return Degrees.of(Math.toDegrees(Math.atan2(y, x)));
                            }));
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
                    turretSubsystem.setAngle(
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
                            }));

            // Hood returns to 0° when not actively shooting — prevents decapitation
            // under the trench. Hood tracking only happens inside aimAndShoot /
            // aimAndShootPass while the driver holds a trigger.
            hoodSubsystem.setDefaultCommand(hoodSubsystem.setAngle(Degrees.of(0)));
        }

        // Let spindexer coast by default. Use the persistent stopHold() default
        // which disables closed-loop control and keeps the duty/voltage at zero
        // while scheduled (the motor idle mode is COAST so it will freewheel).
        spindexerSubsystem.setDefaultCommand(spindexerSubsystem.stopHold());

        // Flywheel pre-spins automatically when the shifted shift is active or within
        // 5 seconds of becoming active — no button required. When the driver holds the
        // right trigger, aimAndShoot preempts this default and tracks the exact speed.
        flywheelSubsystem.setDefaultCommand(
                flywheelSubsystem.setVelocity(
                        () -> {
                            var shift = HubShiftUtil.getShiftedShiftInfo();
                            if (shift.active() || shift.remainingTime() < 5.0) {
                                return FlywheelConstants.kFlywheelDefaultVelocity;
                            }
                            return edu.wpi.first.units.Units.RPM.of(0);
                        }));

        // Ensure intake rollers default to stopped when no command is running —
        // they should not coast, so use the persistent stopHold() default.
        intakeRollersSubsystem.setDefaultCommand(intakeRollersSubsystem.stopHold());

        // Ensure intake pivot holds its commanded setpoint when no one owns it so
        // live tuning and dashboard writes persist.
        intakePivotSubsystem.setDefaultCommand(
                // intakePivotSubsystem.setAngle(() -> intakePivotSubsystem.getPosition()));
                intakePivotSubsystem.set(0));
    }

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
     * <p>Driver controller mappings should be limited to competition-safe behaviors (e.g. aim+shoot
     * on the right trigger). Longer-lived or tuning helpers belong in {@link
     * #configureTestButtonBindings()} and should be enabled only in tuning mode.
     */
    public void configureRealButtonBindings() {
        // Right trigger toggles intake on/off (press once to start, press again to
        // cancel).
        // .rightTrigger().onTrue(intakeSystem.intake());

        // Left trigger immediately stops rollers and holds them stopped while pressed.
        // driverController.leftTrigger().onTrue(intakeSystem.stopRollers());

        driverController
                .rightTrigger()
                .whileTrue(
                        shooterSystem.aimAndShoot(
                                () -> drive.getPose(),
                                () -> drive.getChassisSpeeds(),
                                () -> AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint),
                                3,
                                ShootingLookupTable.Mode.HUB));

        // Left trigger: hold to aim + feed a pass shot. Not shift-gated —
        // feeds freely while held regardless of hub shift state. Turret and hood
        // track the nearest pass target landing zone for the duration.
        driverController
                .leftTrigger()
                .whileTrue(
                        shooterSystem.aimAndShootPass(
                                () -> drive.getPose(), () -> drive.getChassisSpeeds(), 3));

        // Right bumper toggles intake on/off (press once to start, press again to
        // cancel).
        operatorController.rightBumper().onTrue(intakeSystem.intake());

        // Left bumper immediately stops rollers and holds them stopped while pressed.
        operatorController.leftBumper().onTrue(intakeSystem.stopRollers());

        operatorController.a().whileTrue(intakeSystem.agitate()).onFalse(intakeSystem.deploy());
        operatorController.b().whileTrue(shooterSystem.clearShooterSystem());

        operatorController.x().whileTrue(intakeRollersSubsystem.ejectBalls());
        operatorController.y().whileTrue(intakeRollersSubsystem.intakeBalls());

        // Pulse right rumble once per second in the last 5s of an active shift —
        // mirrors 6328's end-of-shift warning. Triggers on remainingTime threshold
        // regardless of active state so the driver is warned before both active→inactive
        // and inactive→active transitions.
        for (int i = 1; i <= 5; i++) {
            final double seconds = i;
            new Trigger(() -> HubShiftUtil.getShiftedShiftInfo().remainingTime() < seconds)
                    .and(RobotModeTriggers.teleop())
                    .onTrue(
                            Commands.runEnd(
                                            () -> driverController.setRumble(RumbleType.kRightRumble, 1.0),
                                            () -> driverController.setRumble(RumbleType.kRightRumble, 0.0))
                                    .withTimeout(0.25));
        }
    }

    /**
     * Enable test/tuning-specific bindings. These are small helpers intended for development and
     * should not be active during normal competition operation. This method should be idempotent in
     * higher-level flows (constructor only currently). Add more bindings here as needed when
     * experimenting.
     */
    public void configureTestButtonBindings() {
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

        driverController.leftBumper().whileTrue(intakePivotSubsystem.set(-.40).withTimeout(0.5));
        driverController.rightBumper().whileTrue(intakeSystem.agitate());

        driverController.rightTrigger().whileTrue(shooterSystem.testShoot());
    }

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

    /** Returns the command selected on the dashboard to run during autonomous. */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}
