package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

/**
 * Shooter lookup tables (embedded defaults).
 *
 * <p>+ * Each row is {distance (m), hood angle (deg), time-of-flight (s)} and uses the project
 * convention where 0° = horizontal.
 */
public final class ShooterLookupTables {
    private ShooterLookupTables() {}

    public static final class LookupRow {
        public final Distance distance;
        public final Angle trajectoryAngle;
        public final Time timeOfFlight;

        public LookupRow(Distance distance, Angle trajectoryAngle, Time timeOfFlight) {
            this.distance = distance;
            this.trajectoryAngle = trajectoryAngle;
            this.timeOfFlight = timeOfFlight;
        }
    }

    public static final LookupRow[] HUB;
    public static final LookupRow[] PASS;

    static {
        // Embedded LUTs: {distance_m, hood_deg, time_of_flight_s}.
        double[][] hubDefault = {
            {1.209, 60.426, 0.45},
            {1.522, 58.926, 0.52},
            {1.809, 57.426, 0.60},
            {2.107, 56.426, 0.68},
            {2.412, 55.426, 0.76},
            {2.738, 53.926, 0.84},
            {3.016, 52.676, 0.92},
            {3.342, 52.426, 1.00},
            {3.666, 50.926, 1.08},
            {3.961, 49.926, 1.16},
            {4.274, 47.926, 1.30},
            {5.005, 47.426, 1.45}
        };
        double[][] passDefault = {
            {1.0, 54.0, 0.35},
            {5.5, 45.0, 1.25}
        };

        double[][] hub = hubDefault;
        double[][] pass = passDefault;

        // Convert raw double arrays into typed lookup rows to expose unit-aware values.
        HUB = new LookupRow[hub.length];
        for (int i = 0; i < hub.length; i++) {
            var r = hub[i];
            HUB[i] = new LookupRow(Meters.of(r[0]), Degrees.of(r[1]), Seconds.of(r[2]));
        }

        PASS = new LookupRow[pass.length];
        for (int i = 0; i < pass.length; i++) {
            var r = pass[i];
            PASS[i] = new LookupRow(Meters.of(r[0]), Degrees.of(r[1]), Seconds.of(r[2]));
        }
    }
}
