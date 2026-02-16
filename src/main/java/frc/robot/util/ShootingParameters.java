package frc.robot.util;

/**
 * Simple, immutable data holder for shooting parameters returned by lookup tables. Contains shooter
 * speed, trajectory angle, and estimated time-of-flight.
 */
public class ShootingParameters {
    /** Shooter wheel speed used by the lookup table (units depend on table; typically RPM). */
    public final double shooterSpeed;

    /** Projectile trajectory launch angle in degrees (as used by lookup tables). */
    public final double trajectoryAngle;

    /** Estimated time-of-flight for the shot, in seconds. */
    public final double timeOfFlight;

    public ShootingParameters(double shooterSpeed, double trajectoryAngle, double timeOfFlight) {
        this.shooterSpeed = shooterSpeed;
        this.trajectoryAngle = trajectoryAngle;
        this.timeOfFlight = timeOfFlight;
    }
}
