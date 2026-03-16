package frc.robot.systems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.geometry.Zones;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class ZoneSystem extends Command {
    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<ChassisSpeeds> speedsSupplier;
    private final ShooterSystem shooterSystem;
    private final Drive drive;
    private final CommandXboxController driverController;

    private DriveMode driveMode = DriveMode.NORMAL;

    private static final double DEADBAND = 0.1;
    private final ProfiledPIDController trenchAngleController;
    private final ProfiledPIDController bumpAngleController;

    @AutoLogOutput private final Trigger inTrench;
    @AutoLogOutput private final Trigger willEnterTrench;
    @AutoLogOutput private final Trigger inBump;
    @AutoLogOutput private final Trigger willEnterBump;
    @AutoLogOutput private final Trigger inAllianceZone;
    @AutoLogOutput private final Trigger inNeutralZone;

    public ZoneSystem(
            Supplier<Pose2d> poseSupplier,
            Supplier<ChassisSpeeds> speedsSupplier,
            ShooterSystem shooterSystem,
            Drive drive,
            CommandXboxController driverController) {
        this.poseSupplier = poseSupplier;
        this.speedsSupplier = speedsSupplier;
        this.shooterSystem = shooterSystem;
        this.drive = drive;
        this.driverController = driverController;
        addRequirements(drive);

        var angleControllerConstraints = new TrapezoidProfile.Constraints(8.0, 20.0);
        trenchAngleController = new ProfiledPIDController(5.0, 0.0, 0.4, angleControllerConstraints);
        trenchAngleController.enableContinuousInput(-Math.PI, Math.PI);
        bumpAngleController = new ProfiledPIDController(5.0, 0.0, 0.4, angleControllerConstraints);
        bumpAngleController.enableContinuousInput(-Math.PI, Math.PI);

        inTrench = Zones.TRENCH_ZONES.contains(poseSupplier).debounce(.1);
        willEnterTrench =
                Zones.TRENCH_ZONES.willContain(
                        poseSupplier, speedsSupplier, DriveConstants.kRotateSnapDuration);
        inBump = Zones.BUMP_ZONES.contains(poseSupplier).debounce(.1);
        willEnterBump =
                Zones.BUMP_ZONES
                        .willContain(poseSupplier, speedsSupplier, DriveConstants.kRotateSnapDuration)
                        .debounce(.1);
        inAllianceZone = Zones.ALLIANCE_ZONES.contains(poseSupplier).debounce(.1);
        inNeutralZone = Zones.NEUTRAL_ZONES.contains(poseSupplier).debounce(.1);

        willEnterTrench.onTrue(updateDriveMode(DriveMode.TRENCH_LOCK));
        willEnterBump.onTrue(updateDriveMode(DriveMode.BUMP_LOCK));

        inTrench.onFalse(updateDriveMode(DriveMode.NORMAL));
        inBump.onFalse(updateDriveMode(DriveMode.NORMAL));
    }

    @Override
    public void execute() {
        // Get linear velocity
        Translation2d linearVelocity =
                getLinearVelocityFromJoysticks(-driverController.getLeftY(), -driverController.getLeftX());

        double omega;
        switch (driveMode) {
            case NORMAL:
                omega = MathUtil.applyDeadband(-driverController.getRightX(), DEADBAND);
                omega = Math.copySign(omega * omega, omega);
                omega *= drive.getMaxAngularSpeedRadPerSec();
                break;
            case TRENCH_LOCK:
                omega =
                        trenchAngleController.calculate(
                                drive.getRotation().getRadians(), getTrenchLockAngle().getRadians());
                break;
            case BUMP_LOCK:
                omega =
                        bumpAngleController.calculate(
                                drive.getRotation().getRadians(), getBumpLockAngle().getRadians());
                break;
            default:
                omega = 0.0;
                break;
        }

        ChassisSpeeds speeds =
                new ChassisSpeeds(
                        linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                        linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                        omega);
        boolean isFlipped =
                DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get() == Alliance.Red;
        drive.runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        speeds,
                        isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));
    }

    private Translation2d getLinearVelocityFromJoysticks(double x, double y) {
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
        Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));
        linearMagnitude = linearMagnitude * linearMagnitude;
        return new Pose2d(Translation2d.kZero, linearDirection)
                .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
                .getTranslation();
    }

    private Rotation2d getTrenchLockAngle() {
        if (Math.abs(MathUtil.inputModulus(drive.getRotation().getDegrees(), -180, 180)) < 90) {
            return Rotation2d.kCCW_90deg;
        } else {
            return Rotation2d.kCW_90deg;
        }
    }

    private Rotation2d getBumpLockAngle() {
        for (int i = -135; i < 180; i += 90) {
            if (Math.abs(MathUtil.inputModulus(drive.getRotation().getDegrees() - i, -180, 180)) <= 45) {
                return Rotation2d.fromDegrees(i);
            }
        }
        return Rotation2d.kZero;
    }

    private Command updateDriveMode(DriveMode newDriveMode) {
        return Commands.runOnce(
                () -> {
                    driveMode = newDriveMode;
                    if (newDriveMode == DriveMode.TRENCH_LOCK) {
                        trenchAngleController.reset(drive.getRotation().getRadians());
                    } else if (newDriveMode == DriveMode.BUMP_LOCK) {
                        bumpAngleController.reset(drive.getRotation().getRadians());
                    }
                });
    }

    /** Periodic logging of zone boundaries to AdvantageScope. */
    public void log() {
        frc.robot.util.geometry.Zones.logAllZones();
    }

    private enum DriveMode {
        NORMAL,
        TRENCH_LOCK,
        BUMP_LOCK
    }
}
