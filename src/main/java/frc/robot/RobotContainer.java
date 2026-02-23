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
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
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
import frc.robot.systems.ShooterSystem;
import frc.robot.util.FuelSim;
import frc.robot.util.RobotMapValidator;
import frc.robot.util.ShootingLookupTable;
import java.util.concurrent.atomic.AtomicReference;

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
    // private final IntakeRollersSubsystem rollersSubsystem = new IntakeRollersSubsystem();
    private final IntakePivotSubsystem intakePivotSubsystem = new IntakePivotSubsystem();
    public FuelSim fuelSim;

    // Tuning state (only used when running in TUNING mode)
    private final AtomicReference<AngularVelocity> tuningFlywheel =
            new AtomicReference<>(ShooterConstants.FlywheelConstants.kFlywheelTargetAngularVelocity);

    // Aggregated shooter system
    private final ShooterSystem shooterSystem =
            new ShooterSystem(
                    flywheelSubsystem, kickerSubsystem, spindexerSubsystem, turretSubsystem, hoodSubsystem);

    // private final IntakeSystem intakeSystem =
    //         new IntakeSystem(rollersSubsystem, intakePivotSubsystem);

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
                {
                    // Real robot, instantiate hardware IO implementations
                    drive =
                            new DriveSubsystem(
                                    new GyroIOPigeon2(),
                                    new ModuleIOTalonFX(TunerConstants.FrontLeft),
                                    new ModuleIOTalonFX(TunerConstants.FrontRight),
                                    new ModuleIOTalonFX(TunerConstants.BackLeft),
                                    new ModuleIOTalonFX(TunerConstants.BackRight));

                    vision =
                            new Vision(
                                    drive::addVisionMeasurement,
                                    new VisionIOPhotonVision(
                                            VisionConstants.cameraFrontName, VisionConstants.robotToCameraLeft),
                                    new VisionIOPhotonVision(
                                            VisionConstants.cameraRightName, VisionConstants.robotToCameraRight),
                                    new VisionIOPhotonVision(
                                            VisionConstants.cameraBackName, VisionConstants.robotToCameraBack));
                    break;
                }
            case TUNING:
                {
                    // TUNING mode: instantiate real hardware IO so tuning runs on the real robot.
                    drive =
                            new DriveSubsystem(
                                    new GyroIOPigeon2(),
                                    new ModuleIOTalonFX(TunerConstants.FrontLeft),
                                    new ModuleIOTalonFX(TunerConstants.FrontRight),
                                    new ModuleIOTalonFX(TunerConstants.BackLeft),
                                    new ModuleIOTalonFX(TunerConstants.BackRight));

                    vision =
                            new Vision(
                                    drive::addVisionMeasurement,
                                    new VisionIOPhotonVision(
                                            VisionConstants.cameraFrontName, VisionConstants.robotToCameraLeft),
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
                }

            default:
                {
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

        // If we're in TUNING mode, schedule a long-running supplier-backed flywheel command
        // that follows the `tuningFlywheel` AtomicReference.
        // if (Constants.currentMode == Constants.Mode.TUNING) {
        //     var tuningCommand = flywheelSubsystem.setVelocity(() -> tuningFlywheel.get());
        //     // Schedule so it runs until we leave TUNING (it requires the flywheel subsystem).
        //     CommandScheduler.getInstance().schedule(tuningCommand);
        //     SmartDashboard.putNumber("Tuning/FlywheelRPM", tuningFlywheel.get().in(RPM));
        // }

        // Configure fuel sim (sim only)
        if (Constants.currentMode == Constants.Mode.SIM) {
            // configureFuelSim();
            // configureFuelSimRobot();
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
                -Inches.of(34.625).div(2.0).plus(Inches.of(11.475556)).in(Meters),
                -Inches.of(34.625).div(2.0).in(Meters),
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
        turretSubsystem.setDefaultCommand(
                turretSubsystem.setAngle(() -> turretSubsystem.getPosition()));
        spindexerSubsystem.setDefaultCommand(spindexerSubsystem.setDutyCycle(0));
        // Have hood hold its current commanded target using the positional controller
        // (we track a commanded target so button bumps are applied relative to it).
        hoodSubsystem.setDefaultCommand(hoodSubsystem.moveToAngle(() -> hoodSubsystem.getTarget()));

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
            case TUNING:
                {
                    // TUNING-mode: simple tuning bindings
                    // Right trigger: hold to run the simple shoot routine (clear-while-spin-up + feed)
                    controller.rightTrigger(0.1).whileTrue(shooterSystem.shoot());

                    // Hood: left/right bumper adjust by -/+1 degree per press.
                    // Use Commands.defer(...) to lazily construct a fresh bump command when the
                    // trigger transitions (avoids capturing a pre-built Command instance or
                    // calling the scheduler directly).
                    controller
                            .leftBumper()
                            .onTrue(shooterSystem.hoodAdjustCommand(Degrees.of(-1.0)).ignoringDisable(true));

                    controller
                            .rightBumper()
                            .onTrue(shooterSystem.hoodAdjustCommand(Degrees.of(1.0)).ignoringDisable(true));

                    // X / Y: decrease/increase tuning RPM by 100
                    controller
                            .x()
                            .onTrue(
                                    Commands.runOnce(
                                                    () -> {
                                                        tuningFlywheel.updateAndGet(prev -> RPM.of(prev.in(RPM) - 100.0));
                                                        SmartDashboard.putNumber(
                                                                "Tuning/FlywheelRPM", tuningFlywheel.get().in(RPM));
                                                    },
                                                    flywheelSubsystem)
                                            .ignoringDisable(true));

                    controller
                            .y()
                            .onTrue(
                                    Commands.runOnce(
                                                    () -> {
                                                        tuningFlywheel.updateAndGet(prev -> RPM.of(prev.in(RPM) + 100.0));
                                                        SmartDashboard.putNumber(
                                                                "Tuning/FlywheelRPM", tuningFlywheel.get().in(RPM));
                                                    },
                                                    flywheelSubsystem)
                                            .ignoringDisable(true));

                    // A/B: snap turret to +90/-90 degrees
                    controller.a().onTrue(turretSubsystem.setAngle(Degrees.of(90)));
                    controller.b().onTrue(turretSubsystem.setAngle(Degrees.of(-90)));

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
}
