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
import frc.robot.constants.Constants.Mode;
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
import frc.robot.util.AllianceFlipUtil;
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
    // Guard to ensure test bindings are only enabled once
    private boolean tuningBindingsEnabled = false;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Validate RobotMap wiring early at startup and warn if duplicate IDs are
        // found.
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

                    // vision =
                    // new Vision(
                    // drive::addVisionMeasurement,
                    // new VisionIOPhotonVision(
                    // VisionConstants.cameraFrontName,
                    // VisionConstants.robotToCameraLeft),
                    // new VisionIOPhotonVision(
                    // VisionConstants.cameraRightName,
                    // VisionConstants.robotToCameraRight),
                    // new VisionIOPhotonVision(
                    // VisionConstants.cameraBackName,
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
                    // vision =
                    // new Vision(
                    // drive::addVisionMeasurement,
                    // new VisionIOPhotonVisionSim(
                    // VisionConstants.cameraFrontName,
                    // VisionConstants.robotToCameraLeft,
                    // drive::getPose),
                    // new VisionIOPhotonVisionSim(
                    // VisionConstants.cameraRightName,
                    // VisionConstants.robotToCameraRight,
                    // drive::getPose),
                    // new VisionIOPhotonVisionSim(
                    // VisionConstants.cameraBackName,
                    // VisionConstants.robotToCameraBack,
                    // drive::getPose));

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
                    // vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new
                    // VisionIO() {});

                    break;
                }
        }
        configureButtonBindings();
    }

    /**
     * Enable test/tuning-specific bindings. Safe to call multiple times (idempotent). Intended to be
     * called from {@link Robot#testInit()} so DriverStation test mode can activate tuning bindings at
     * runtime.
     */
    public void enableTestBindings() {
        // Only enable once
        if (tuningBindingsEnabled) {
            return;
        }
        tuningBindingsEnabled = true;

        // Bind the test/tuning controls that were previously guarded by a compile-time tuning flag.
        bindFlywheelVelocity(driverController);
        driverController
                .leftTrigger()
                .whileTrue(
                        intakeRollersSubsystem.setVelocity(IntakeConstants.Rollers.kTargetAngularVelocity));

        bindHoodDuty(operatorController);
        bindTurretPovs(operatorController);

        operatorController.a().whileTrue(intakePivotSubsystem.set(-0.10));
        operatorController.b().whileTrue(intakePivotSubsystem.set(0.10));
    }

    // Helper that binds flywheel right trigger to the prep velocity and stops on release.
    private void bindFlywheelVelocity(CommandXboxController controller) {
        controller
                .rightTrigger()
                .whileTrue(
                        flywheelSubsystem.setVelocity(
                                ShooterConstants.FlywheelConstants.kFlywheelPrepAngularVelocity))
                .whileFalse(flywheelSubsystem.stopNow());
    }

    // Helper that binds a controller's triggers to hood duty controls.
    private void bindHoodDuty(CommandXboxController controller) {
        controller.rightTrigger().whileTrue(hoodSubsystem.setDutyCycle(0.10));
        controller.leftTrigger().whileTrue(hoodSubsystem.setDutyCycle(-0.10));
    }

    // Helper that binds POV directions to turret angle presets on the supplied controller.
    private void bindTurretPovs(CommandXboxController controller) {
        controller.povLeft().onTrue(turretSubsystem.setAngle(Degrees.of(90)));
        controller.povUpLeft().onTrue(turretSubsystem.setAngle(Degrees.of(45)));
        controller.povUp().onTrue(turretSubsystem.setAngle(Degrees.of(0)));
        controller.povUpRight().onTrue(turretSubsystem.setAngle(Degrees.of(-45)));
        controller.povRight().onTrue(turretSubsystem.setAngle(Degrees.of(-90)));
    }

    // Centralized reset-gyro command so multiple bindings can reuse the same behavior.
    private Command resetGyroZeroCommand() {
        return Commands.runOnce(
                        () ->
                                drive.setPose(
                                        AllianceFlipUtil.apply(
                                                new Pose2d(drive.getPose().getTranslation(), new Rotation2d()))),
                        drive)
                .ignoringDisable(true);
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
                        () -> -driverController.getLeftY(),
                        () -> -driverController.getLeftX(),
                        () -> -driverController.getRightX()));

        kickerSubsystem.setDefaultCommand(kickerSubsystem.setDutyCycle(0));
        turretSubsystem.setDefaultCommand(
                turretSubsystem.setAngle(() -> turretSubsystem.getPosition()));
        spindexerSubsystem.setDefaultCommand(spindexerSubsystem.setDutyCycle(0));
        // Ensure flywheel holds zero when no one owns it so releasing buttons returns
        // it to idle
        flywheelSubsystem.setDefaultCommand(flywheelSubsystem.stopNow());

        // Ensure intake rollers default to stopped when no command is running
        intakeRollersSubsystem.setDefaultCommand(intakeRollersSubsystem.setDutyCycle(0));

        // Ensure intake pivot holds its commanded setpoint when no one owns it so
        // live tuning and dashboard writes persist.
        intakePivotSubsystem.setDefaultCommand(
                // intakePivotSubsystem.setAngle(() -> intakePivotSubsystem.getPosition()));
                intakePivotSubsystem.set(0));

        // Have hood hold its current commanded target using the positional controller
        hoodSubsystem.setDefaultCommand(hoodSubsystem.setAngle(hoodSubsystem.getPosition()));

        if (Constants.getMode() == Mode.SIM) {

            // SIM: keep the same SIM bindings as the test/tuning bindings (applies only in SIM
            // mode when test bindings are not activated via DriverStation)
            bindFlywheelVelocity(driverController);
            bindHoodDuty(driverController);
            bindTurretPovs(driverController);

        } else if (Constants.getMode() == Mode.REAL) {

            // REAL: right trigger holds aim+shoot (uses live odometry); left trigger stops
            // controller
            driverController
                    .rightTrigger()
                    .whileTrue(
                            shooterSystem.aimAndShoot(
                                    () -> drive.getPose(),
                                    () -> new edu.wpi.first.math.kinematics.ChassisSpeeds(0.0, 0.0, 0.0),
                                    () ->
                                            AllianceFlipUtil.apply(
                                                    frc.robot.constants.FieldConstants.Hub.innerCenterPoint),
                                    3,
                                    frc.robot.util.ShootingLookupTable.Mode.HUB));

            // Left trigger remains a manual stop if needed
            driverController.leftTrigger().onTrue(shooterSystem.stopShooting());
        } else {
            // Default (REPLAY/unknown) — no platform-specific bindings here.

        }

        // Reset gyro to 0° when Back button is pressed (available in both REAL and SIM)
        driverController.start().onTrue(resetGyroZeroCommand());

        // START button intentionally removed — Back now performs the same reset on all modes.
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
