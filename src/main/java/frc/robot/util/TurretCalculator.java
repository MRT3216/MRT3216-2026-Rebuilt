// Physics-based turret shot calculator adapted from
// https://github.com/hammerheads5000/2026Rebuilt/blob/main/src/main/java/frc/robot/subsystems/turret/TurretCalculator.java
// Improvements over the original:
//   - Uses MRT3216's FieldConstants and ShooterConstants directly
//   - Integrates with ShootingLookupTable instead of a custom TreeMap
//   - Early-exit convergence for iterative methods mirrors HybridTurretUtil
package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.constants.ShooterConstants.FlywheelConstants.kWheelDiameter;
import static frc.robot.constants.ShooterConstants.TurretConstants.kRobotToTurretTransform;
import static frc.robot.constants.ShooterConstants.TurretConstants.kSoftLimitMax;
import static frc.robot.constants.ShooterConstants.TurretConstants.kSoftLimitMin;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.constants.FieldConstants;
import frc.robot.util.shooter.ShootingLookupTable;
import org.littletonrobotics.junction.Logger;

public class TurretCalculator {

    // Compression / slip factor for flywheel → ball-speed conversion.
    // Accounts for the ball deforming against the wheel during contact.
    // Source: hammerheads5000 empirical tuning.
    private static final double COMPRESSION_FACTOR = 0.54;

    /** Flywheel radius derived from configured wheel diameter. */
    private static final double FLYWHEEL_RADIUS_M = kWheelDiameter.in(Meters) / 2.0;

    // ── Core geometry helpers ────────────────────────────────────────────────────

    /**
     * Horizontal distance (metres) from the robot centre to a field target. Uses the 2D projection of
     * the 3D target position.
     */
    public static Distance getDistanceToTarget(Pose2d robot, Translation3d target) {
        return Meters.of(robot.getTranslation().getDistance(target.toTranslation2d()));
    }

    // ── Flywheel speed conversions ───────────────────────────────────────────────

    /**
     * Convert a ball exit linear velocity to the required flywheel angular velocity. Accounts for
     * wheel compression via {@code COMPRESSION_FACTOR}. See: <a
     * href="https://www.desmos.com/geometry/l4edywkmha">Desmos derivation</a>
     */
    public static AngularVelocity linearToAngularVelocity(LinearVelocity vel) {
        return RadiansPerSecond.of(vel.in(MetersPerSecond) / FLYWHEEL_RADIUS_M / COMPRESSION_FACTOR);
    }

    /**
     * Convert a flywheel angular velocity to the expected ball exit linear velocity. Accounts for
     * wheel compression via {@code COMPRESSION_FACTOR}.
     */
    public static LinearVelocity angularToLinearVelocity(AngularVelocity vel) {
        return MetersPerSecond.of(vel.in(RadiansPerSecond) * FLYWHEEL_RADIUS_M * COMPRESSION_FACTOR);
    }

    // ── Time-of-flight ───────────────────────────────────────────────────────────

    /**
     * Time for a projectile to travel a horizontal distance at the given exit velocity and hood
     * angle.
     *
     * <p>hoodAngle here follows the class convention: 0 = vertical, PI/2 = horizontal. The horizontal
     * velocity component is therefore {@code vel * sin(hoodAngle)}.
     */
    public static Time calculateTimeOfFlight(
            LinearVelocity exitVelocity, Angle hoodAngle, Distance distance) {
        double vel = exitVelocity.in(MetersPerSecond);
        // sin because hoodAngle is from vertical; sin(hoodAngle) = cos(elevation)
        double horizontalVel = vel * Math.sin(hoodAngle.in(Radians));
        if (Math.abs(horizontalVel) < 1e-6) return Seconds.of(Double.MAX_VALUE);
        return Seconds.of(distance.in(Meters) / horizontalVel);
    }

    // ── Azimuth calculation ──────────────────────────────────────────────────────

    /**
     * Robot-relative turret azimuth needed to point at a 3D field target. Prefers the closest valid
     * angle within the turret's soft limits; falls back to the ±360° equivalent if the primary
     * solution is out of range.
     *
     * @param robot current robot pose
     * @param target 3D field target (only XY used for azimuth)
     * @param currentAngle current turret angle (used to pick the closest solution)
     * @return robot-relative turret azimuth in Rotations
     */
    public static Angle calculateAzimuthAngle(
            Pose2d robot, Translation3d target, Angle currentAngle) {
        Translation2d turretXY =
                new Pose3d(robot).transformBy(kRobotToTurretTransform).toPose2d().getTranslation();
        Translation2d dir = target.toTranslation2d().minus(turretXY);
        return calculateAzimuthAngle(robot, dir.getAngle().getMeasure(), currentAngle);
    }

    /** Robot-relative turret azimuth from a field-relative bearing angle. */
    public static Angle calculateAzimuthAngle(
            Pose2d robot, Angle fieldRelativeAngle, Angle currentAngle) {
        double angle =
                MathUtil.inputModulus(
                        new Rotation2d(fieldRelativeAngle).minus(robot.getRotation()).getRotations(),
                        -0.5,
                        0.5);
        double current = currentAngle.in(Rotations);
        // Prefer the ±1 rotation equivalent if it keeps us within soft limits
        if (current > 0 && angle + 1 <= kSoftLimitMax.in(Rotations)) angle += 1;
        if (current < 0 && angle - 1 >= kSoftLimitMin.in(Rotations)) angle -= 1;
        Logger.recordOutput("TurretCalculator/DesiredAzimuthRot", angle);
        return Rotations.of(angle);
    }

    // ── Target prediction ────────────────────────────────────────────────────────

    /**
     * Predict where a (possibly stationary) target will appear to be after {@code timeOfFlight}
     * seconds when the robot is moving at {@code fieldSpeeds}.
     *
     * <p>Because we subtract robot velocity × time, a faster robot effectively "moves the target" in
     * the opposite direction – i.e., the turret must lead the actual hub position.
     */
    public static Translation3d predictTargetPos(
            Translation3d target, ChassisSpeeds fieldSpeeds, Time timeOfFlight) {
        double t = timeOfFlight.in(Seconds);
        return new Translation3d(
                target.getX() - fieldSpeeds.vxMetersPerSecond * t,
                target.getY() - fieldSpeeds.vyMetersPerSecond * t,
                target.getZ());
    }

    // ── Iterative moving-shot (lookup table) ─────────────────────────────────────
    public static ShotData iterativeMovingShotFromMap(
            Pose2d robot,
            ChassisSpeeds fieldSpeeds,
            Translation3d target,
            int iterations,
            ShootingLookupTable table) {

        Distance distance = getDistanceToTarget(robot, target);
        var params = table.getParameters(distance);
        Time tof = table.getTimeOfFlight(distance);

        // Build initial ShotData from table values.
        // params.trajectoryAngle is elevation (deg, 0=horizontal); convert to hoodAngle convention.
        ShotData shot =
                new ShotData(
                        params.shooterSpeed, // already AngularVelocity in RPS
                        Radians.of(Math.PI / 2.0 - params.trajectoryAngle.in(Radians)),
                        target);
        Translation3d predictedTarget = target;

        for (int i = 0; i < iterations; i++) {
            predictedTarget = predictTargetPos(target, fieldSpeeds, tof);
            distance = getDistanceToTarget(robot, predictedTarget);
            params = table.getParameters(distance);
            tof = table.getTimeOfFlight(distance);
            shot =
                    new ShotData(
                            params.shooterSpeed,
                            Radians.of(Math.PI / 2.0 - params.trajectoryAngle.in(Radians)),
                            predictedTarget);
        }
        return shot;
    }

    // ── ShotData record ──────────────────────────────────────────────────────────

    public record ShotData(double exitVelocity, double hoodAngle, Translation3d target) {

        /** Construct from typed units (preferred). */
        public ShotData(AngularVelocity exitVelocity, Angle hoodAngle, Translation3d target) {
            this(exitVelocity.in(RadiansPerSecond), hoodAngle.in(Radians), target);
        }

        /** Construct with the default blue hub target. */
        public ShotData(AngularVelocity exitVelocity, Angle hoodAngle) {
            this(exitVelocity, hoodAngle, FieldConstants.HUB_BLUE);
        }

        /** Ball exit linear velocity (accounts for flywheel compression). */
        public LinearVelocity getLinearExitVelocity() {
            return angularToLinearVelocity(RadiansPerSecond.of(exitVelocity));
        }

        /** Flywheel angular exit velocity as a typed measure. */
        public AngularVelocity getAngularExitVelocity() {
            return RadiansPerSecond.of(exitVelocity);
        }

        /** Hood angle from vertical as a typed measure. */
        public Angle getHoodAngle() {
            return Radians.of(hoodAngle);
        }

        /** Elevation angle from horizontal (inverse of hoodAngle convention). */
        public Angle getElevationAngle() {
            return Radians.of(Math.PI / 2.0 - hoodAngle);
        }

        /** Predicted target this solution was computed for. */
        public Translation3d getTarget() {
            return target;
        }

        /**
         * Linearly interpolate between two ShotData instances. Useful for smoothly blending between
         * table entries during tuning.
         */
        public static ShotData interpolate(ShotData a, ShotData b, double t) {
            return new ShotData(
                    MathUtil.interpolate(a.exitVelocity, b.exitVelocity, t),
                    MathUtil.interpolate(a.hoodAngle, b.hoodAngle, t),
                    b.target);
        }
    }
}
