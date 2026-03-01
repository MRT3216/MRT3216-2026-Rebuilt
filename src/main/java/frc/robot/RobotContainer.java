// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.constants.Constants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ShooterConstants;
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
import frc.robot.systems.IntakeSystem;
import frc.robot.systems.ShooterSystem;
import frc.robot.util.RobotMapValidator;
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
    // private final Vision vision;
    private final FlywheelSubsystem flywheelSubsystem = new FlywheelSubsystem();
    private final KickerSubsystem kickerSubsystem = new KickerSubsystem();
    private final TurretSubsystem turretSubsystem = new TurretSubsystem();
    private final SpindexerSubsystem spindexerSubsystem = new SpindexerSubsystem();
    private final HoodSubsystem hoodSubsystem = new HoodSubsystem();
    private final IntakePivotSubsystem intakePivotSubsystem = new IntakePivotSubsystem();
    private final IntakeRollersSubsystem rollersSubsystem = new IntakeRollersSubsystem();

    // Aggregated shooter system
    private final ShooterSystem shooterSystem =
            new ShooterSystem(
                    flywheelSubsystem, kickerSubsystem, spindexerSubsystem, turretSubsystem, hoodSubsystem);

    private final IntakeSystem intakeSystem =
            new IntakeSystem(rollersSubsystem, intakePivotSubsystem);

    // Controller
    private final CommandXboxController controller = new CommandXboxController(0);

    // Dashboard inputs (initialized by setupAutoChooser when enabled)
    private LoggedDashboardChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Validate RobotMap wiring early at startup and warn if duplicate IDs are
        // found.
        RobotMapValidator.validate();
        switch (Constants.currentMode) {
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

                    //     vision =
                    //             new Vision(
                    //                     drive::addVisionMeasurement,
                    //                     new VisionIOPhotonVision(
                    //                             VisionConstants.cameraFrontName,
                    // VisionConstants.robotToCameraLeft),
                    //                     new VisionIOPhotonVision(
                    //                             VisionConstants.cameraRightName,
                    // VisionConstants.robotToCameraRight),
                    //                     new VisionIOPhotonVision(
                    //                             VisionConstants.cameraBackName,
                    // VisionConstants.robotToCameraBack));

                    break;
                }

            case TUNING:
                {
                    // TUNING mode: instantiate real hardware IO so tuning runs on the real robot.
                    drive =
                            new Drive(
                                    new GyroIO() {},
                                    new ModuleIOTalonFX(TunerConstants.FrontLeft),
                                    new ModuleIOTalonFX(TunerConstants.FrontRight),
                                    new ModuleIOTalonFX(TunerConstants.BackLeft),
                                    new ModuleIOTalonFX(TunerConstants.BackRight));

                    //     vision =
                    //             new Vision(
                    //                     drive::addVisionMeasurement,
                    //                     new VisionIOPhotonVision(
                    //                             VisionConstants.cameraFrontName,
                    // VisionConstants.robotToCameraLeft),
                    //                     new VisionIOPhotonVision(
                    //                             VisionConstants.cameraRightName,
                    // VisionConstants.robotToCameraRight),
                    //                     new VisionIOPhotonVision(
                    //                             VisionConstants.cameraBackName,
                    // VisionConstants.robotToCameraBack));

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
                    // Sim robot, instantiate physics sim IO implementations
                    //     vision =
                    //             new Vision(
                    //                     drive::addVisionMeasurement,
                    //                     new VisionIOPhotonVisionSim(
                    //                             VisionConstants.cameraFrontName,
                    //                             VisionConstants.robotToCameraLeft,
                    //                             drive::getPose),
                    //                     new VisionIOPhotonVisionSim(
                    //                             VisionConstants.cameraRightName,
                    //                             VisionConstants.robotToCameraRight,
                    //                             drive::getPose),
                    //                     new VisionIOPhotonVisionSim(
                    //                             VisionConstants.cameraBackName,
                    //                             VisionConstants.robotToCameraBack,
                    //                             drive::getPose));

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
                    // vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});

                    break;
                }
        }
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(
                DriveCommands.joystickDrive(
                        drive,
                        () -> -controller.getLeftY(),
                        () -> -controller.getLeftX(),
                        () -> -controller.getRightX()));

        kickerSubsystem.setDefaultCommand(kickerSubsystem.setDutyCycle(0));
        turretSubsystem.setDefaultCommand(
                turretSubsystem.setAngle(() -> turretSubsystem.getPosition()));
        spindexerSubsystem.setDefaultCommand(spindexerSubsystem.setDutyCycle(0));
        // Ensure flywheel holds zero when no one owns it so releasing buttons returns it to idle
        flywheelSubsystem.setDefaultCommand(flywheelSubsystem.stopHold());
        // Ensure intake rollers default to stopped when no command is running
        rollersSubsystem.setDefaultCommand(rollersSubsystem.setDutyCycle(0));
        // Ensure intake pivot holds its commanded setpoint when no one owns it so
        // live tuning and dashboard writes persist.
        intakePivotSubsystem.setDefaultCommand(
                intakePivotSubsystem.setAngle(() -> intakePivotSubsystem.getTarget()));
        // Have hood hold its current commanded target using the positional controller
        // (we track a commanded target so button bumps are applied relative to it).
        hoodSubsystem.setDefaultCommand(hoodSubsystem.moveToAngle(() -> hoodSubsystem.getTarget()));

        // Bind X to a different command depending on runtime mode: SIM uses a
        // simplified routine,
        // REAL uses the dynamic aim-and-shoot routine (requires pose/vision suppliers).
        switch (Constants.currentMode) {
            case REAL:
                {
                    // REAL: right trigger holds aim+shoot (uses live odometry); left trigger stops
                    controller
                            .rightTrigger()
                            .whileTrue(
                                    shooterSystem.aimAndShoot(
                                            () -> drive.getPose(),
                                            () -> new edu.wpi.first.math.kinematics.ChassisSpeeds(0.0, 0.0, 0.0),
                                            () -> frc.robot.constants.FieldConstants.Hub.innerCenterPoint,
                                            3,
                                            frc.robot.util.ShootingLookupTable.Mode.HUB));

                    // Left trigger remains a manual stop if needed
                    controller.leftTrigger().onTrue(shooterSystem.stopShooting());

                    break;
                }

            case SIM:
            case TUNING:
                {
                    // TUNING/SIM: hold right trigger to run adjustable-target shooting (A/B bumps apply live)
                    controller.rightTrigger().whileTrue(shooterSystem.startShootingWithAdjustableTarget());

                    controller.leftTrigger().onTrue(shooterSystem.stopShooting());

                    // Hood: left/right bumper adjust by -/+1 degree per press.
                    controller
                            .leftBumper()
                            .onTrue(shooterSystem.hoodAdjustCommand(Degrees.of(-1.0)).ignoringDisable(true));

                    controller
                            .rightBumper()
                            .onTrue(shooterSystem.hoodAdjustCommand(Degrees.of(1.0)).ignoringDisable(true));

                    // A/B/X/Y: quick run buttons for velocity subsystems (TUNING/SIM)
                    // X -> Intake Rollers
                    controller
                            .x()
                            .whileTrue(
                                    rollersSubsystem.setVelocity(IntakeConstants.Rollers.kTargetAngularVelocity));

                    // Y -> Spindexer
                    controller
                            .y()
                            .whileTrue(
                                    spindexerSubsystem.setVelocity(
                                            ShooterConstants.SpindexerConstants.kSpindexerTargetAngularVelocity));

                    // A/B: small RPM bumps for tuning (do not require subsystems so they
                    // won't interrupt running shooting pipelines). Use onTrue so a single
                    // press performs a one-shot bump.
                    controller.a().onTrue(shooterSystem.bumpFlywheelDown(50));
                    controller.b().onTrue(shooterSystem.bumpFlywheelUp(50));

                    controller.povLeft().onTrue(turretSubsystem.setAngle(Degrees.of(90)));
                    controller.povUp().onTrue(turretSubsystem.setAngle(Degrees.of(0)));
                    controller.povRight().onTrue(turretSubsystem.setAngle(Degrees.of(-90)));

                    break;
                }

            default:
                {
                    // Default (REPLAY/unknown) — no SIM-specific bindings here.
                    break;
                }
        }

        // Reset gyro to 0° when Back button is pressed (available in both REAL and SIM)
        controller
                .back()
                .onTrue(
                        Commands.runOnce(
                                        () ->
                                                drive.setPose(
                                                        new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                                        drive)
                                .ignoringDisable(true));

        // START button resets gyro when running on the real robot or in TUNING mode.
        if (Constants.currentMode == Constants.Mode.REAL
                || Constants.currentMode == Constants.Mode.TUNING) {
            controller
                    .start()
                    .onTrue(
                            Commands.runOnce(
                                            () ->
                                                    drive.setPose(
                                                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                                            drive)
                                    .ignoringDisable(true));
        }
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
    @SuppressWarnings("unused")
    private void setupAutoChooser() {
        // Set up auto routines using AdvantageKit's LoggedDashboardChooser backed by
        // an AutoBuilder. This mirrors the original project behavior.
        // Note: This method is intentionally not called by default. Call it from
        // the constructor to enable the auto chooser at runtime.
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    }
}
