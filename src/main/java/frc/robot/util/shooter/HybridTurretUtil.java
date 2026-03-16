package frc.robot.util.shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.constants.ShooterConstants.TurretConstants.kRobotToTurretTransform;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

/** Utility for turret shooting solutions while the robot is in motion. */
public class HybridTurretUtil {
    public static ShotSolution computeMovingShot(
            Pose2d robotPose,
            ChassisSpeeds fieldSpeeds,
            Translation3d target,
            int iterations,
            Distance convergenceEpsilon,
            ShootingLookupTable table) {
        // Use the turret origin (robot -> turret transform) when measuring distance/azimuth so
        // the model accounts for the physical offset between robot center and turret.
        // The configured transform is a pure translation so we can compute the turret world XY by
        // rotating the offset into field coordinates rather than creating Pose3d objects.
        double theta = robotPose.getRotation().getRadians();
        double ox = kRobotToTurretTransform.getTranslation().getX();
        double oy = kRobotToTurretTransform.getTranslation().getY();
        double turretX = robotPose.getX() + ox * Math.cos(theta) - oy * Math.sin(theta);
        double turretY = robotPose.getY() + ox * Math.sin(theta) + oy * Math.cos(theta);
        var turretXY = new Translation2d(turretX, turretY);
        double initialDistMeters = turretXY.getDistance(target.toTranslation2d());
        var initialDist = Meters.of(initialDistMeters);
        var tofTime = table.getTimeOfFlight(initialDist);
        double tof = tofTime.in(Seconds);

        Translation3d predictedTarget = target;
        Distance leadDist = Meters.of(initialDistMeters);
        Distance prevLeadDist = leadDist;
        for (int i = 0; i < iterations; i++) {
            double pX = target.getX() - fieldSpeeds.vxMetersPerSecond * tof;
            double pY = target.getY() - fieldSpeeds.vyMetersPerSecond * tof;
            predictedTarget = new Translation3d(pX, pY, target.getZ());

            double leadDistMeters = turretXY.getDistance(predictedTarget.toTranslation2d());
            leadDist = Meters.of(leadDistMeters);
            tof = table.getTimeOfFlight(leadDist).in(Seconds);

            if (Math.abs(leadDist.in(Meters) - prevLeadDist.in(Meters)) < convergenceEpsilon.in(Meters)) {
                break;
            }
            prevLeadDist = leadDist;
        }

        // Compute azimuth from the turret origin, not the robot center.
        var dir = predictedTarget.toTranslation2d().minus(turretXY);
        Angle azimuthRobot =
                Radians.of(Math.atan2(dir.getY(), dir.getX()) - robotPose.getRotation().getRadians());

        Distance min = table.getMinDistance();
        Distance max = table.getMaxDistance();

        Distance clamped = leadDist;
        if (min != null && leadDist.in(Meters) < min.in(Meters)) clamped = min;
        if (max != null && leadDist.in(Meters) > max.in(Meters)) clamped = max;

        var finalParams = table.getParameters(clamped);
        boolean isValid = clamped == leadDist;

        return new ShotSolution(
                clamped,
                azimuthRobot,
                finalParams.trajectoryAngle,
                finalParams.shooterSpeed,
                finalParams.timeOfFlight,
                isValid);
    }

    public record ShotSolution(
            Distance leadDistance,
            Angle turretAzimuth,
            Angle hoodAngle,
            AngularVelocity flywheelSpeed,
            Time timeOfFlight,
            boolean isValid) {}
}
