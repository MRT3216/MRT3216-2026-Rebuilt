package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

/**
 * Shooter lookup tables (embedded defaults).
 *
 * <p>Each row is {distance (m), hood angle (deg), time-of-flight (s)} and uses the project
 * convention where 0° = horizontal.
 */
public final class ShooterLookupTables {
    private ShooterLookupTables() {}

    /** Immutable row: distance, hood/trajectory angle, and time-of-flight. */
    public record LookupRow(Distance distance, Angle trajectoryAngle, Time timeOfFlight) {}

    public static final LookupRow[] HUB;
    public static final LookupRow[] PASS;

    static {
        // Embedded LUTs: {distance_m, hood_deg, time_of_flight_s}.
        double[][] hubDefault = {
            {1.5494, 0, 0.82},
            {2.82575, 3.1, 1.37},
            {4.10845, 7.47, 1.585},
            {5.8039, 10.8, 1.705},
            {6.7437, 12.65, 1.94},
            {7.4041, 12.8, 2.14}
        };
        double[][] passDefault = {
            {3.5, 0.0, 1.3},
            {5.0, 1.0, 1.6},
            {6.5, 2.0, 1.9},
            {8.0, 3.0, 2.2},
            {10.0, 4.0, 2.5}
        };

        // Convert raw double arrays into typed lookup rows to expose unit-aware values.
        HUB = new LookupRow[hubDefault.length];
        for (int i = 0; i < hubDefault.length; i++) {
            var r = hubDefault[i];
            HUB[i] = new LookupRow(Meters.of(r[0]), Degrees.of(r[1]), Seconds.of(r[2]));
        }

        PASS = new LookupRow[passDefault.length];
        for (int i = 0; i < passDefault.length; i++) {
            var r = passDefault[i];
            PASS[i] = new LookupRow(Meters.of(r[0]), Degrees.of(r[1]), Seconds.of(r[2]));
        }
    }
}
