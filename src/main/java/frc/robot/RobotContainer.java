// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
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
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.systems.IntakeSystem;
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
    private final Drive drive;
    private final Vision vision;
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
                            new Drive(
                                    new GyroIO() {},
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
                            new Drive(
                                    new GyroIO() {},
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
                            new Drive(
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
        // duty. Flywheel default will re-apply the currently applied setpoint each loop
        // so
        // the PID controller remains active; callers manually clear or change the
        // setpoint.
        // flywheelSubsystem.setDefaultCommand(
        // flywheelSubsystem.setVelocity(() -> flywheelSubsystem.getAppliedSetpoint()));
        kickerSubsystem.setDefaultCommand(kickerSubsystem.setDutyCycle(0));
        turretSubsystem.setDefaultCommand(
                turretSubsystem.setAngle(() -> turretSubsystem.getPosition()));
        spindexerSubsystem.setDefaultCommand(spindexerSubsystem.setDutyCycle(0));
        // Ensure intake rollers default to stopped when no command is running
        rollersSubsystem.setDefaultCommand(rollersSubsystem.setDutyCycle(0));
        // Have hood hold its current commanded target using the positional controller
        // (we track a commanded target so button bumps are applied relative to it).
        hoodSubsystem.setDefaultCommand(hoodSubsystem.moveToAngle(() -> hoodSubsystem.getTarget()));

        // Bind X to a different command depending on runtime mode: SIM uses a
        // simplified routine,
        // REAL uses the dynamic aim-and-shoot routine (requires pose/vision suppliers).
        switch (Constants.currentMode) {
            case REAL:
                {
                    // // Start/stop buttons: X starts the long-running shoot pipeline; Y stops it.
                    // Command start =
                    // shooterSystem.startShooting(
                    // () -> new Pose2d(),
                    // () -> new ChassisSpeeds(0.0, 0.0, 0.0),
                    // () -> FieldConstants.Hub.innerCenterPoint,
                    // 3,
                    // ShootingLookupTable.Mode.HUB);
                    // controller.x().onTrue(start);
                    // controller.y().onTrue(shooterSystem.stopShooting());
                    // break;
                }

            case SIM:
            case TUNING:
                {
                    // TUNING-mode: simple tuning bindings
                    // Right trigger: press to start the shooting pipeline (fixed prep speed)
                    controller.rightTrigger().onTrue(shooterSystem.startShooting());
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

                    // A -> Kicker
                    controller
                            .a()
                            .whileTrue(
                                    kickerSubsystem.setVelocity(
                                            ShooterConstants.KickerConstants.kKickerTargetAngularVelocity));

                    // B -> Flywheel
                    controller
                            .b()
                            .whileTrue(
                                    flywheelSubsystem.setVelocity(
                                            ShooterConstants.FlywheelConstants.kFlywheelPrepAngularVelocity))
                            .whileFalse(flywheelSubsystem.stopFlywheel());

                    // Previously we snapped the turret on A/B in SIM/TUNING; keep the commands
                    // commented out here in case we want to re-enable that behavior later.
                    // controller.a().onTrue(turretSubsystem.setAngle(Degrees.of(90)));
                    // controller.b().onTrue(turretSubsystem.setAngle(Degrees.of(-90)));

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
