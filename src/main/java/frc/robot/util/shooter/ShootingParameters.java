package frc.robot.util.shooter;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;

/** Unit-aware, immutable data holder for shooting parameters returned by lookup tables. */
public class ShootingParameters {
    public final AngularVelocity shooterSpeed;
    public final Angle trajectoryAngle;
    public final Time timeOfFlight;

    public ShootingParameters(
            AngularVelocity shooterSpeed, Angle trajectoryAngle, Time timeOfFlight) {
        this.shooterSpeed = shooterSpeed;
        this.trajectoryAngle = trajectoryAngle;
        this.timeOfFlight = timeOfFlight;
    }
}
