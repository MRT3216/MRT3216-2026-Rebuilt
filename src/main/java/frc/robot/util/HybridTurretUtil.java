package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

/**
 * Utility class for calculating high-fidelity turret shooting solutions while the robot is in
 * motion. It accounts for robot velocity to "lead" the target, ensuring the ball reaches the
 * objective based on the time of flight and relative field movement.
 *
 * <p>This class performs an iterative refinement that predicts target motion during the ball's
 * flight and looks up tuned table values to return a complete shot solution.
 */
public class HybridTurretUtil {

    /**
     * Calculate a shot solution while the robot is moving.
     *
     * <p>It computes the required turret azimuth, hood angle, and flywheel speed to hit a target
     * while accounting for the robot's motion. The method performs a small number of iterative
     * refinements to predict where the target will be during the ball's flight and looks up tuned
     * table parameters for the final solution.
     *
     * @param robotPose The current global pose of the robot (from odometry).
     * @param fieldSpeeds The current velocity of the robot relative to the field (m/s).
     * @param target The 3D coordinates of the target (Hub or Ally) in field space.
     * @param iterations Number of refinement passes (typically 2-4). More iterations increase
     *     accuracy but consume more CPU.
     * @param table The shooting lookup table used to retrieve tuned parameters.
     * @return A ShotSolution containing all setpoints for the shooter and turret subsystems.
     */
    public static ShotSolution computeMovingShot(
            Pose2d robotPose,
            ChassisSpeeds fieldSpeeds,
            Translation3d target,
            int iterations,
            Distance convergenceEpsilon,
            ShootingLookupTable table) {

        // 1. Initial Guess: Calculate distance to the target as if we were stationary.
        double initialDist = robotPose.getTranslation().getDistance(target.toTranslation2d());
        double tof = table.getTimeOfFlight(initialDist);

        Translation3d predictedTarget = target;
        double leadDist = initialDist;

        // 2. Iterative Refinement:
        // We calculate where the target "appears" to move due to our own velocity
        // during the ball's flight.
        // We then re-calculate Time of Flight (ToF) based on that new distance and
        // repeat.
        double prevLeadDist = leadDist;
        for (int i = 0; i < iterations; i++) {
            // Subtracting velocity * time effectively "pushes" the target in the opposite
            // direction
            // of our travel, allowing the shooter to lead the shot correctly.
            double pX = target.getX() - fieldSpeeds.vxMetersPerSecond * tof;
            double pY = target.getY() - fieldSpeeds.vyMetersPerSecond * tof;
            predictedTarget = new Translation3d(pX, pY, target.getZ());

            // Update parameters for the next iteration based on the "virtual" lead distance
            leadDist = robotPose.getTranslation().getDistance(predictedTarget.toTranslation2d());
            tof = table.getTimeOfFlight(leadDist);

            // Early-exit: if the change in lead distance between iterations is below the
            // convergence threshold, break out to save CPU. Convert the units-aware
            // convergenceEpsilon to meters for the comparison.
            if (Math.abs(leadDist - prevLeadDist) < convergenceEpsilon.in(Meters)) {
                break;
            }
            prevLeadDist = leadDist;
        }

        // 3. Final Solve: Look up tuned RPS/Angle values for the final calculated lead
        // distance.
        var params = table.getParameters(leadDist);

        // Calculate the relative angle (azimuth) from the robot's current heading to
        // the predicted target.
        double dx = predictedTarget.getX() - robotPose.getX();
        double dy = predictedTarget.getY() - robotPose.getY();
        Angle azimuth = Radians.of(Math.atan2(dy, dx) - robotPose.getRotation().getRadians());

        return new ShotSolution(
                leadDist,
                azimuth,
                Degrees.of(params.trajectoryAngle),
                RotationsPerSecond.of(params.shooterSpeed),
                Seconds.of(params.timeOfFlight),
                (leadDist > 0.5 && leadDist < 15.0) // Validates shot is within physically possible range
                );
    }

    /**
     * Container record for all parameters required to execute a shot.
     *
     * @param leadDistanceMeters The calculated distance to the "virtual" target point.
     * @param turretAzimuth The required rotation for the turret relative to the robot chassis.
     * @param hoodAngle The required vertical angle for the adjustable hood.
     * @param flywheelSpeed The target rotational velocity for the shooter wheels.
     * @param timeOfFlight Estimated time (seconds) for the ball to reach the target.
     * @param isValid False if the target is too close or too far to be reliably hit.
     */
    public record ShotSolution(
            double leadDistanceMeters,
            Angle turretAzimuth,
            Angle hoodAngle,
            AngularVelocity flywheelSpeed,
            Time timeOfFlight,
            boolean isValid) {}
}
