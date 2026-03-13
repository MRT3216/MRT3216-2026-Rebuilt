// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.constants.IntakeConstants.Rollers.kTargetAngularVelocity;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Mode;
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
import frc.robot.util.RobotMapValidator;
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

    // private final ZoneSystem zoneSystem;

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

                    // zoneSystem =
                    // new ZoneSystem(
                    // drive::getPose, drive::getChassisSpeeds, shooterSystem, drive,
                    // driverController);

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

                    // zoneSystem =
                    // new ZoneSystem(
                    // drive::getPose, drive::getChassisSpeeds, shooterSystem, drive,
                    // driverController);

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
                    vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
                    // zoneSystem =
                    // new ZoneSystem(
                    // drive::getPose, drive::getChassisSpeeds, shooterSystem, drive,
                    // driverController);
                    break;
                }
        }
        // Auto for shooting the first eight balls
        // Need to use the pathplanner to create the name and pull it through the dashboard
        NamedCommands.registerCommand(
                "FireEight",
                shooterSystem
                        .aimAndShoot(
                                drive::getPose,
                                drive::getChassisSpeeds,
                                () -> AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint),
                                3,
                                ShootingLookupTable.Mode.HUB)
                        .withTimeout(10.0));

        setupAutoChooser();
        configureDefaultCommands();
        configureButtonBindings();
    }

    public void configureDefaultCommands() {
        // drive.setDefaultCommand(zoneSystem);
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
        turretSubsystem.setDefaultCommand(
                turretSubsystem.setAngle(() -> turretSubsystem.getPosition()));
        // turretSubsystem.setAngle(Degrees.of(0)));

        // Let spindexer coast by default. Use the persistent stopHold() default
        // which disables closed-loop control and keeps the duty/voltage at zero
        // while scheduled (the motor idle mode is COAST so it will freewheel).
        spindexerSubsystem.setDefaultCommand(spindexerSubsystem.stopHold());

        // Let flywheel coast by default rather than forcing a zero setpoint.
        flywheelSubsystem.setDefaultCommand(flywheelSubsystem.stopHold());

        // Ensure intake rollers default to stopped when no command is running —
        // they should not coast, so use the persistent stopHold() default.
        intakeRollersSubsystem.setDefaultCommand(intakeRollersSubsystem.stopHold());

        // Ensure intake pivot holds its commanded setpoint when no one owns it so
        // live tuning and dashboard writes persist.
        intakePivotSubsystem.setDefaultCommand(
                // intakePivotSubsystem.setAngle(() -> intakePivotSubsystem.getPosition()));
                intakePivotSubsystem.set(0));

        // Have hood hold its current commanded target using the positional controller
        hoodSubsystem.setDefaultCommand(hoodSubsystem.setAngle(hoodSubsystem.getPosition()));
    }

    private void configureButtonBindings() {
        if (Constants.tuningMode) {
            configureTestButtonBindings();
        } else if (Constants.getMode() == Mode.SIM) {
            configureRealButtonBindings();
        } else if (Constants.getMode() == Mode.REAL) {
            configureRealButtonBindings();
        }

        // Reset gyro to 0° when the Start button is pressed (available in both REAL and
        // SIM). Use the controller "start()" binding here intentionally — if you prefer
        // Back
        // change the binding to driverController.back().
        driverController.start().onTrue(resetGyroZeroCommand());
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
        driverController.rightTrigger().onTrue(intakeSystem.intake());

        // Left trigger immediately stops rollers and holds them stopped while pressed.
        driverController.leftTrigger().onTrue(intakeSystem.stopRollers());

        // REAL: right trigger holds aim+shoot (uses live odometry); left trigger stops
        // controller.
        operatorController
                .rightTrigger()
                .onTrue(
                        shooterSystem.aimAndShoot(
                                () -> drive.getPose(),
                                () -> drive.getChassisSpeeds(),
                                () -> AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint),
                                3,
                                ShootingLookupTable.Mode.HUB));

        // Left trigger remains a manual stop if needed
        operatorController.leftTrigger().onTrue(shooterSystem.interruptShooting());

        // Right bumper toggles intake on/off (press once to start, press again to
        // cancel).
        operatorController.rightBumper().onTrue(intakeSystem.intake());

        // Left bumper immediately stops rollers and holds them stopped while pressed.
        operatorController.leftBumper().onTrue(intakeSystem.stopRollers());

        operatorController.a().whileTrue(intakeSystem.agitate());

        operatorController.b().whileTrue(shooterSystem.clearShooterSystem());

        operatorController.x().whileTrue(intakeRollersSubsystem.ejectBalls());
        operatorController.y().whileTrue(intakeRollersSubsystem.intakeBalls());
    }

    /**
     * Enable test/tuning-specific bindings. These are small helpers intended for development and
     * should not be active during normal competition operation. This method should be idempotent in
     * higher-level flows (constructor only currently). Add more bindings here as needed when
     * experimenting.
     */
    public void configureTestButtonBindings() {
        driverController.a().whileTrue(flywheelSubsystem.setToTunedVelocity());
        driverController.b().whileTrue(intakeRollersSubsystem.setVelocity(kTargetAngularVelocity));

        driverController.x().whileTrue(spindexerSubsystem.feedShooter());
        driverController.y().whileTrue(kickerSubsystem.feedShooter());

        driverController.leftBumper().whileTrue(intakePivotSubsystem.set(-.40).withTimeout(0.5));
        driverController
                .rightBumper()
                .whileTrue(
                        Commands.repeatingSequence(
                                intakePivotSubsystem
                                        .set(0.15)
                                        .withTimeout(0.15)
                                        .andThen(intakePivotSubsystem.set(-0.15).withTimeout(0.15))));

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
     * AutoChooser helper: creates the dashboard auto chooser.
     *
     * <p>Disabled by default. To enable, call {@code setupAutoChooser()} from the constructor and
     * uncomment the implementation below.
     */
    private void setupAutoChooser() {
        // "Auto Chooser" matches the topic key the Elastic dashboard subscribes to.
        autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser());

        // Fire all 8 pre-loaded balls from the starting position — no driving.
        autoChooser.addOption(
                "Fire 8 Balls",
                shooterSystem
                        .aimAndShoot(
                                drive::getPose,
                                drive::getChassisSpeeds,
                                () -> AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint),
                                3,
                                ShootingLookupTable.Mode.HUB)
                        .withTimeout(13.0));
    }

    /** Returns the command selected on the dashboard to run during autonomous. */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}
