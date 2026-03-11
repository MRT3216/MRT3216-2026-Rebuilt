package frc.robot.util.shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
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
        double initialDistMeters = robotPose.getTranslation().getDistance(target.toTranslation2d());
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

            double leadDistMeters =
                    robotPose.getTranslation().getDistance(predictedTarget.toTranslation2d());
            leadDist = Meters.of(leadDistMeters);
            tof = table.getTimeOfFlight(leadDist).in(Seconds);

            if (Math.abs(leadDist.in(Meters) - prevLeadDist.in(Meters)) < convergenceEpsilon.in(Meters)) {
                break;
            }
            prevLeadDist = leadDist;
        }

        double dxRobot = predictedTarget.getX() - robotPose.getX();
        double dyRobot = predictedTarget.getY() - robotPose.getY();
        Angle azimuthRobot =
                Radians.of(Math.atan2(dyRobot, dxRobot) - robotPose.getRotation().getRadians());

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
