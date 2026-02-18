// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.KickerSubsystem;
import frc.robot.subsystems.shooter.SpindexerSubsystem;
import frc.robot.subsystems.shooter.TurretSubsystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.systems.ShooterSystem;
import frc.robot.util.FuelSim;
import frc.robot.util.RobotMapValidator;
import frc.robot.util.ShootingLookupTable;

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
    private final DriveSubsystem drive;
    private final Vision vision;
    private final FlywheelSubsystem flywheelSubsystem = new FlywheelSubsystem();
    private final KickerSubsystem kickerSubsystem = new KickerSubsystem();
    private final TurretSubsystem turretSubsystem = new TurretSubsystem();
    private final SpindexerSubsystem spindexerSubsystem = new SpindexerSubsystem();
    private final HoodSubsystem hoodSubsystem = new HoodSubsystem();
    public FuelSim fuelSim;

    // Aggregated shooter system
    private final ShooterSystem shooterSystem =
            new ShooterSystem(
                    flywheelSubsystem, kickerSubsystem, spindexerSubsystem, turretSubsystem, hoodSubsystem);

    // Controller
    private final CommandXboxController controller = new CommandXboxController(0);

    // Dashboard inputs
    // private final LoggedDashboardChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Validate RobotMap wiring early at startup and warn if duplicate IDs are
        // found.
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
                drive =
                        new DriveSubsystem(
                                new GyroIOPigeon2() {},
                                new ModuleIOSim(TunerConstants.FrontLeft),
                                new ModuleIOSim(TunerConstants.FrontRight),
                                new ModuleIOSim(TunerConstants.BackLeft),
                                new ModuleIOSim(TunerConstants.BackRight));
                // Sim robot, instantiate physics sim IO implementations
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

                break;

            default:
                // Replayed robot, disable IO implementations
                drive =
                        new DriveSubsystem(
                                new GyroIO() {},
                                new ModuleIO() {},
                                new ModuleIO() {},
                                new ModuleIO() {},
                                new ModuleIO() {});

                // (Use same number of dummy implementations as the real robot)
                vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
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

        // Configure fuel sim (sim only)
        if (Constants.currentMode == Constants.Mode.SIM) {
            configureFuelSim();
            configureFuelSimRobot();
        }
    }

    private void configureFuelSim() {
        fuelSim = new FuelSim();
        fuelSim.spawnStartingFuel();

        fuelSim.start();
        SmartDashboard.putData(
                Commands.runOnce(
                                () -> {
                                    fuelSim.clearFuel();
                                    fuelSim.spawnStartingFuel();
                                })
                        .withName("Reset Fuel")
                        .ignoringDisable(true));
    }

    private void configureFuelSimRobot() {
        fuelSim.registerRobot(
                Inches.of(34.625).in(Meters),
                Inches.of(34.625).in(Meters),
                Inches.of(5.125).in(Meters),
                drive::getPose,
                drive::getChassisSpeeds);
        fuelSim.registerIntake(
                Inches.of(34.625).div(2).in(Meters),
                Inches.of(34.625).div(2).plus(Inches.of(11.475556)).in(Meters),
                -Inches.of(34.625).div(2.0).in(Meters),
                Inches.of(34.625).div(2.0).in(Meters),
                () -> true);
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

        // Default commands now use closed-loop controllers (PID) instead of open-loop
        // duty
        flywheelSubsystem.setDefaultCommand(flywheelSubsystem.setDutyCycle(0));
        kickerSubsystem.setDefaultCommand(kickerSubsystem.setDutyCycle(0));
        // turretSubsystem.setDefaultCommand(turretSubsystem.setDutyCycle(0));
        spindexerSubsystem.setDefaultCommand(spindexerSubsystem.setDutyCycle(0));
        // Have hood hold its current position using the positional controller
        hoodSubsystem.setDefaultCommand(hoodSubsystem.setAngle(() -> hoodSubsystem.getPosition()));

        // Schedule `setVelocity` when the Xbox controller's B button is pressed,
        // cancelling on release.
        // Button A spins a low test speed using closed-loop velocity control
        // controller
        //         .a()
        //
        // .whileTrue(flywheelSubsystem.setVelocity(Constants.ShooterConstants.kLowSpinVelocity));

        controller
                .a().onTrue(
                        shooterSystem.aimAndShoot(
                                drive::getPose,
                                drive::getChassisSpeeds,
                                () -> vision.getTagTranslation3d(10),
                                3,
                                ShootingLookupTable.Mode.HUB));

        // Button B spins to tuned shooting speed
        controller
                .b()
                .whileTrue(flywheelSubsystem.setVelocity(Constants.ShooterConstants.kTargetFlywheel));

        // controller.b().whileTrue(turretSubsystem.setAngle(Degrees.of(90)));
        // controller.a().whileTrue(turretSubsystem.setAngle(Degrees.of(-90)));
        // Bind X to a different command depending on runtime mode: SIM uses a
        // simplified routine,
        // REAL uses the dynamic aim-and-shoot routine (requires pose/vision suppliers).
        switch (Constants.currentMode) {
            case REAL:
                {
                    // Start/stop buttons: X starts the long-running shoot pipeline; Y stops it.
                    Command start =
                            shooterSystem.startShooting(
                                    () -> new Pose2d(),
                                    () -> new ChassisSpeeds(0.0, 0.0, 0.0),
                                    () -> FieldConstants.Hub.innerCenterPoint,
                                    3,
                                    ShootingLookupTable.Mode.HUB);
                    controller.x().onTrue(start);
                    controller.y().onTrue(shooterSystem.stopShooting());
                    break;
                }
            case SIM:
            default:
                {
                    Command simShoot = shooterSystem.shoot();
                    controller.x().whileTrue(simShoot);
                    break;
                }
        }
        // Hood presets and manual control
        controller.leftBumper().onTrue(hoodSubsystem.setAngle(Degrees.of(15)));
        controller.rightBumper().onTrue(hoodSubsystem.setAngle(Degrees.of(45)));
        // Manual hood control: small incremental adjustments to the target angle while
        // held.
        // The supplier computes an absolute angle based on current position and trigger
        // axis.
        controller
                .leftTrigger(0.1)
                .whileTrue(
                        hoodSubsystem.setAngle(
                                () ->
                                        Degrees.of(
                                                hoodSubsystem.getPosition().in(Degrees)
                                                        - controller.getLeftTriggerAxis() * 2.0)));
        controller
                .rightTrigger(0.1)
                .whileTrue(
                        hoodSubsystem.setAngle(
                                () ->
                                        Degrees.of(
                                                hoodSubsystem.getPosition().in(Degrees)
                                                        + controller.getRightTriggerAxis() * 2.0)));
        // Quick manual feed: only start the short feed if turret and hood are at their
        // setpoints.
        Command guardedQuickFeed =
                Commands.waitUntil(turretSubsystem.atSetpoint.and(hoodSubsystem.atSetpoint))
                        .withTimeout(Seconds.of(0.5))
                        .andThen(
                                kickerSubsystem
                                        .setVelocity(Constants.KickerConstants.kTargetVelocity)
                                        .withTimeout(Seconds.of(1))
                                        .andThen(
                                                spindexerSubsystem.setVelocity(
                                                        Constants.SpindexerConstants.kTargetVelocity)));

        controller.y().whileTrue(guardedQuickFeed);
        controller
                .back()
                .whileTrue(
                        turretSubsystem.setAngle(
                                () ->
                                        Degrees.of(
                                                turretSubsystem.getPosition().in(Degrees) + controller.getLeftX() * 5.0)));

        // Press left-stick to snap turret to -90°, press right-stick to snap to +90°.
        controller.leftStick().onTrue(turretSubsystem.setAngle(Degrees.of(-90)));
        controller.rightStick().onTrue(turretSubsystem.setAngle(Degrees.of(90)));

        // Test button: while the START button is held, run the shooter's clear routine
        // (spins kicker and spindexer in reverse at the configured clear RPMs).
        controller.start().whileTrue(shooterSystem.clear());

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
     * return autoChooser.get();
     * }
     */
}
