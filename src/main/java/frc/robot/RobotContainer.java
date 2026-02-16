// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.Constants;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.KickerSubsystem;
import frc.robot.subsystems.shooter.SpindexerSubsystem;
import frc.robot.systems.ShooterSystem;
import frc.robot.util.RobotMapValidator;

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
    // private final DriveSubsystem drive;
    // private final Vision vision;
    private final FlywheelSubsystem flywheelSubsystem = new FlywheelSubsystem();
    private final KickerSubsystem kickerSubsystem = new KickerSubsystem();
    // private final TurretSubsystem turretSubsystem = new TurretSubsystem();
    private final SpindexerSubsystem spindexerSubsystem = new SpindexerSubsystem();

    // Aggregated shooter system
    private final ShooterSystem shooterSystem =
            new ShooterSystem(flywheelSubsystem, kickerSubsystem, spindexerSubsystem);

    // Controller
    private final CommandXboxController controller = new CommandXboxController(0);

    // Dashboard inputs
    // private final LoggedDashboardChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Validate RobotMap wiring early at startup and warn if duplicate IDs are found.
        RobotMapValidator.validate();
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
                // a CANcoder
                // drive =
                // new DriveSubsystem(
                // new GyroIOPigeon2(),
                // new ModuleIOTalonFX(TunerConstants.FrontLeft),
                // new ModuleIOTalonFX(TunerConstants.FrontRight),
                // new ModuleIOTalonFX(TunerConstants.BackLeft),
                // new ModuleIOTalonFX(TunerConstants.BackRight));

                // // Real robot, instantiate hardware IO implementations
                // // vision =
                // // new Vision(
                // // drive::addVisionMeasurement,
                // // new VisionIOLimelight(cameraLeftName, drive::getRotation),
                // // new VisionIOLimelight(cameraRightName, drive::getRotation));
                // vision =
                // new Vision(
                // drive::addVisionMeasurement,
                // new VisionIOPhotonVision(cameraLeftName, robotToCameraLeft),
                // new VisionIOPhotonVision(cameraRightName, robotToCameraRight),
                // new VisionIOPhotonVision(cameraBackName, robotToCameraBack));
                // break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                // drive =
                // new DriveSubsystem(
                // new GyroIO() {},
                // new ModuleIOSim(TunerConstants.FrontLeft),
                // new ModuleIOSim(TunerConstants.FrontRight),
                // new ModuleIOSim(TunerConstants.BackLeft),
                // new ModuleIOSim(TunerConstants.BackRight));

                // Sim robot, instantiate physics sim IO implementations
                // vision =
                // new Vision(
                // drive::addVisionMeasurement,
                // new VisionIOPhotonVisionSim(cameraLeftName, robotToCameraLeft,
                // drive::getPose),
                // new VisionIOPhotonVisionSim(cameraRightName, robotToCameraRight,
                // drive::getPose),
                // new VisionIOPhotonVisionSim(cameraBackName, robotToCameraBack,
                // drive::getPose));
                break;

            default:
                // Replayed robot, disable IO implementations
                // drive =
                // new DriveSubsystem(
                // new GyroIO() {},
                // new ModuleIO() {},
                // new ModuleIO() {},
                // new ModuleIO() {},
                // new ModuleIO() {});

                // (Use same number of dummy implementations as the real robot)
                // vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new
                // VisionIO() {});
                break;
        }

        // Set up auto routines
        // autoChooser = new LoggedDashboardChooser<>("Auto Choices",
        // AutoBuilder.buildAutoChooser());

        // // // Set up SysId routines
        // autoChooser.addOption(
        // "Drive Wheel Radius Characterization",
        // DriveCommands.wheelRadiusCharacterization(drive));
        // autoChooser.addOption(
        // "Drive Simple FF Characterization",
        // DriveCommands.feedforwardCharacterization(drive));
        // autoChooser.addOption(
        // "Drive SysId (Quasistatic Forward)",
        // drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // autoChooser.addOption(
        // "Drive SysId (Quasistatic Reverse)",
        // drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // autoChooser.addOption(
        // "Drive SysId (Dynamic Forward)",
        // drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // autoChooser.addOption(
        // "Drive SysId (Dynamic Reverse)",
        // drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // Configure the button bindings
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
        // drive.setDefaultCommand(
        // DriveCommands.joystickDrive(
        // drive,
        // () -> -controller.getLeftY(),
        // () -> -controller.getLeftX(),
        // () -> -controller.getRightX()));

        flywheelSubsystem.setDefaultCommand(flywheelSubsystem.setDutyCycle(0));
        kickerSubsystem.setDefaultCommand(kickerSubsystem.setDutyCycle(0));
        // turretSubsystem.setDefaultCommand(turretSubsystem.setDutyCycle(0));
        spindexerSubsystem.setDefaultCommand(spindexerSubsystem.setDutyCycle(0));

        // Schedule `setVelocity` when the Xbox controller's B button is pressed,
        // cancelling on release.
        controller.a().whileTrue(flywheelSubsystem.setDutyCycle(0.2));
        controller.b().whileTrue(flywheelSubsystem.setVelocity(RPM.of(3000)));

        // controller.b().whileTrue(turretSubsystem.setAngle(Degrees.of(90)));
        // controller.a().whileTrue(turretSubsystem.setAngle(Degrees.of(-90)));
        controller
                .rightTrigger()
                .whileTrue(flywheelSubsystem.setDutyCycle(() -> controller.getRightTriggerAxis()));
        controller.x().whileTrue(shooterSystem.shoot());
        controller
                .y()
                .whileTrue(
                        kickerSubsystem
                                .setVelocity(RPM.of(2000))
                                .withTimeout(Seconds.of(1))
                                .andThen(spindexerSubsystem.setVelocity(RPM.of(4000))));
        controller
                .leftTrigger()
                .whileTrue(kickerSubsystem.setDutyCycle(() -> controller.getRightTriggerAxis()));

        // // Lock to 0° when A button is held
        // controller
        // .a()
        // .whileTrue(
        // DriveCommands.joystickDriveAtAngle(
        // drive,
        // () -> -controller.getLeftY(),
        // () -> -controller.getLeftX(),
        // () -> Rotation2d.kZero));

        // // Switch to X pattern when X button is pressed
        // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

        // // Reset gyro to 0° when B button is pressed
        // controller
        // .b()
        // .onTrue(
        // Commands.runOnce(
        // () ->
        // drive.setPose(
        // new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
        // drive)
        // .ignoringDisable(true));
    }

    /*
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * (Method currently commented out: kept here for future reference.)
     *
     * Example (uncomment to enable):
     * public Command getAutonomousCommand() {
     *     return autoChooser.get();
     * }
     */
}
