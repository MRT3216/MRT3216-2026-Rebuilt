package frc.robot.util.shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.constants.ShooterConstants.TurretConstants.kRobotToTurretTransform;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

/**
 * Iterative moving-shot solver for a turret-based shooter.
 *
 * <p>Given the robot's pose, field-relative chassis velocity, and a 3-D target point, this class
 * computes a motion-compensated {@link ShotSolution} that accounts for the ball inheriting the
 * robot's velocity at launch. The solver iterates: at each step it adjusts the virtual aim point by
 * {@code −velocity × timeOfFlight} (so the ball's inherited drift cancels out), re-computes the
 * lead distance, and looks up a new time-of-flight from the provided {@link ShootingLookupTable}.
 * Iteration stops when the lead distance converges within a configurable epsilon or the iteration
 * cap is reached.
 *
 * <p>All distance and azimuth calculations are performed from the <em>turret origin</em> (the
 * physical turret position on the robot, defined by {@code kRobotToTurretTransform}), not the robot
 * center.
 */
public final class HybridTurretUtil {
    private HybridTurretUtil() {}

    /**
     * Compute a motion-compensated shot solution.
     *
     * @param robotPose current field-relative robot pose
     * @param fieldSpeeds field-relative chassis velocity (used for lead compensation)
     * @param target 3-D field-relative target point (e.g. hub center or pass landing zone)
     * @param iterations maximum number of convergence iterations
     * @param convergenceEpsilon distance change threshold below which the solver stops early
     * @param table lookup table to use for hood angle, flywheel speed, and time-of-flight
     * @return a fully populated {@link ShotSolution}
     */
    public static ShotSolution computeMovingShot(
            Pose2d robotPose,
            ChassisSpeeds fieldSpeeds,
            Translation3d target,
            int iterations,
            Distance convergenceEpsilon,
            ShootingLookupTable table) {

        // --- Turret world-space origin ------------------------------------------
        // The configured transform is a pure translation, so we rotate the offset
        // into field coordinates with simple trig (avoids Pose3d allocations).
        double theta = robotPose.getRotation().getRadians();
        double ox = kRobotToTurretTransform.getTranslation().getX();
        double oy = kRobotToTurretTransform.getTranslation().getY();
        double turretX = robotPose.getX() + ox * Math.cos(theta) - oy * Math.sin(theta);
        double turretY = robotPose.getY() + ox * Math.sin(theta) + oy * Math.cos(theta);

        double targetX = target.getX();
        double targetY = target.getY();

        // --- Initial (stationary) distance & TOF --------------------------------
        double dx = targetX - turretX;
        double dy = targetY - turretY;
        double leadDistM = Math.hypot(dx, dy);
        double tofS = table.getTimeOfFlight(Meters.of(leadDistM)).in(Seconds);

        // --- Iterative lead-compensation loop -----------------------------------
        // The ball inherits the robot's field-velocity at launch.  To compensate,
        // we shift the virtual aim point AGAINST robot motion so that the ball's
        // inherited drift carries it back onto the true target.
        double vx = fieldSpeeds.vxMetersPerSecond;
        double vy = fieldSpeeds.vyMetersPerSecond;
        double epsilonM = convergenceEpsilon.in(Meters);
        double prevLeadDistM = leadDistM;

        double predictedX = targetX;
        double predictedY = targetY;

        for (int i = 0; i < iterations; i++) {
            predictedX = targetX - vx * tofS;
            predictedY = targetY - vy * tofS;

            dx = predictedX - turretX;
            dy = predictedY - turretY;
            leadDistM = Math.hypot(dx, dy);
            tofS = table.getTimeOfFlight(Meters.of(leadDistM)).in(Seconds);

            if (Math.abs(leadDistM - prevLeadDistM) < epsilonM) {
                break;
            }
            prevLeadDistM = leadDistM;
        }

        // --- Turret azimuth (robot-relative) ------------------------------------
        Angle azimuth = Radians.of(Math.atan2(predictedY - turretY, predictedX - turretX) - theta);

        // --- LUT lookup with clamping -------------------------------------------
        double clampedM = leadDistM;
        var min = table.getMinDistance();
        var max = table.getMaxDistance();
        if (min.isPresent()) clampedM = Math.max(clampedM, min.get().in(Meters));
        if (max.isPresent()) clampedM = Math.min(clampedM, max.get().in(Meters));

        boolean isValid = (clampedM == leadDistM);
        var clampedDist = Meters.of(clampedM);
        var finalParams = table.getParameters(clampedDist);

        return new ShotSolution(
                clampedDist, azimuth, finalParams.trajectoryAngle(), finalParams.timeOfFlight(), isValid);
    }

    /**
     * Immutable result of a moving-shot computation.
     *
     * @param leadDistance motion-compensated distance from turret to virtual aim point (clamped to
     *     LUT bounds)
     * @param turretAzimuth robot-relative azimuth the turret should point
     * @param hoodAngle hood/trajectory angle from the LUT
     * @param timeOfFlight estimated ball flight time from the LUT
     * @param isValid {@code true} when the lead distance fell within the LUT range (not clamped)
     */
    public record ShotSolution(
            Distance leadDistance,
            Angle turretAzimuth,
            Angle hoodAngle,
            Time timeOfFlight,
            boolean isValid) {}
}
