package frc.robot.util;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;

/**
 * Unit-aware, immutable data holder for shooting parameters returned by lookup tables. Fields use
 * WPILib unit types to avoid unit-mixing bugs.
 */
public class ShootingParameters {
    /** Shooter wheel speed used by the lookup table (rotations per second). */
    public final AngularVelocity shooterSpeed;

    /** Projectile trajectory launch angle. */
    public final Angle trajectoryAngle;

    /** Estimated time-of-flight for the shot. */
    public final Time timeOfFlight;

    public ShootingParameters(
            AngularVelocity shooterSpeed, Angle trajectoryAngle, Time timeOfFlight) {
        this.shooterSpeed = shooterSpeed;
        this.trajectoryAngle = trajectoryAngle;
        this.timeOfFlight = timeOfFlight;
    }
}
