package frc.robot.util;

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.Constants.TurretConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.*;
import frc.robot.constants.FieldConstants;

/**
 * Advanced targeting utility for the 2026 Season.
 *
 * <p>Integrated with the official AprilTag field layout to solve 3D projectile kinematics and
 * moving-target compensation.
 */
public class TurretCalculator {

    /** Calculates the horizontal 2D distance to a 3D field target. */
    public static Distance getDistanceToTarget(Pose2d robot, Translation3d target) {
        return Meters.of(robot.getTranslation().getDistance(target.toTranslation2d()));
    }

    /**
     * Solves for the required launch angle (Hood/Pivot) to hit a 3D target.
     *
     * <p>Note: Uses kTurretOffsetZ to account for the physical mounting height on the robot.
     */
    public static Angle calculateAngleFromVelocity(
            Pose2d robot, LinearVelocity velocity, Translation3d target) {
        double g = frc.robot.constants.Constants.PhysicsConstants.kStandardGravity; // gravity in m/s^2
        double vel = velocity.in(MetersPerSecond);
        double x_dist = getDistanceToTarget(robot, target).in(Meters);

        // Adjust target Z relative to the turret pivot height
        double y_dist = target.getZ() - kTurretOffsetZ.in(Meters);

        // Standard Projectile Formula: tan(theta) = (v^2 ± sqrt(v^4 - g(gx^2 + 2yv^2))) / gx
        double determinant = Math.pow(vel, 4) - g * (g * x_dist * x_dist + 2 * y_dist * vel * vel);

        if (determinant < 0) return Radians.of(0);

        double angle = Math.atan(((vel * vel) + Math.sqrt(determinant)) / (g * x_dist));
        return Radians.of(angle);
    }

    /** Calculates the turret azimuth (rotation) relative to the robot's heading. */
    public static Angle calculateAzimuthAngle(Pose2d robot, Translation3d target) {
        // Find the turret's field-space position using the robot-to-turret transform
        Translation2d turretTranslation =
                new Pose3d(robot).transformBy(kRobotToTurretTransform).toPose2d().getTranslation();

        Translation2d direction = target.toTranslation2d().minus(turretTranslation);

        return Radians.of(
                MathUtil.inputModulus(
                        direction.getAngle().minus(robot.getRotation()).getRadians(), 0, 2 * Math.PI));
    }

    /**
     * Solves for a trajectory that clears the 2026 Funnel obstacle.
     *
     * <p>Pulls dimensions directly from {@link FieldConstants.Hub}.
     */
    public static ShotData calculateShotFromFunnelClearance(
            Pose2d robot, Translation3d actualTarget, Translation3d predictedTarget) {

        double x_dist = getDistanceToTarget(robot, predictedTarget).in(Meters);
        double y_dist = predictedTarget.getZ() - kTurretOffsetZ.in(Meters);

        double g = frc.robot.constants.Constants.PhysicsConstants.kStandardGravity;
        double r = FieldConstants.Hub.innerWidth / 2.0; // Radius based on official Hub inner width
        double h = FieldConstants.Hub.innerHeight + kDistanceAboveFunnel.in(Meters);

        // Algebra for solving a parabola passing through the clearance point and target
        double A1 = x_dist * x_dist;
        double B1 = x_dist;
        double D1 = y_dist;
        double A2 = -x_dist * x_dist + Math.pow(x_dist - r, 2);
        double B2 = -r;
        double D2 = h;
        double Bm = -B2 / B1;
        double A3 = Bm * A1 + A2;
        double D3 = Bm * D1 + D2;
        double a = D3 / A3;
        double b = (D1 - A1 * a) / B1;

        double theta = Math.atan(b);
        double v0 = Math.sqrt(-g / (2 * a * Math.pow(Math.cos(theta), 2)));

        if (Double.isNaN(v0) || Double.isNaN(theta)) {
            return new ShotData(MetersPerSecond.of(0), Radians.of(0), predictedTarget);
        }
        return new ShotData(MetersPerSecond.of(v0), Radians.of(theta), predictedTarget);
    }

    /** Standard shot container for logging and subsystem control. */
    public record ShotData(LinearVelocity exitVelocity, Angle hoodAngle, Translation3d target) {
        public LinearVelocity getExitVelocity() {
            return exitVelocity;
        }

        public Angle getHoodAngle() {
            return hoodAngle;
        }
    }
}
