package frc.robot.util.shooter;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;

/**
 * Immutable data holder for LUT shooting parameters: hood angle and time-of-flight.
 *
 * <p>Flywheel RPM is intentionally excluded — it always comes from {@link ShooterModel}, not the
 * lookup table.
 */
public record ShootingParameters(Angle trajectoryAngle, Time timeOfFlight) {}
