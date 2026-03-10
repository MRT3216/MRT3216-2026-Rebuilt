package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.robot.constants.ShooterLookupTables;
import java.util.Comparator;
import java.util.TreeMap;

/** Shooting lookup table: loads embedded constants and provides unit-aware interpolation. */
public class ShootingLookupTable {
    public enum Mode {
        HUB,
        PASS
    }

    // Use Distance keys for stronger type-safety. Compare by meters for ordering.
    private final TreeMap<Distance, ShootingParameters> lookupTable =
            new TreeMap<>(Comparator.comparingDouble(d -> d.in(Meters)));

    /**
     * Create a ShootingLookupTable that loads the requested dataset from Constants.
     *
     * @param mode the dataset mode to load (HUB or PASS)
     */
    public ShootingLookupTable(Mode mode) {
        loadFromConstants(mode);
    }

    private void loadFromConstants(Mode mode) {
        ShooterLookupTables.LookupRow[] data;
        switch (mode) {
            case HUB:
                data = ShooterLookupTables.HUB;
                break;
            case PASS:
                data = ShooterLookupTables.PASS;
                break;
            default:
                data = new ShooterLookupTables.LookupRow[0];
        }

        for (var row : data) {
            addEntry(row.distance, row.trajectoryAngle, row.timeOfFlight);
        }
    }

    /**
     * Interpolate and return shooting parameters for a given distance.
     *
     * @param distance target distance as a {@link Distance}
     * @return interpolated shooting parameters
     */
    public ShootingParameters getParameters(Distance distance) {
        // Find nearest keys for angle/ToF interpolation. Even if the requested distance
        // exactly matches a LUT key, we still want to use the ShooterModel for the
        // flywheel speed (RPM) per project decision.
        Distance lowerKey = lookupTable.floorKey(distance);
        Distance upperKey = lookupTable.ceilingKey(distance);

        if (lowerKey == null && upperKey == null) {
            // Empty table: return model speed with NaN angle/tof
            var modelSpeed = ShooterModel.flywheelSpeedForDistance(distance);
            return new ShootingParameters(modelSpeed, Degrees.of(Double.NaN), Seconds.of(Double.NaN));
        }

        if (lowerKey == null) {
            var upper = lookupTable.get(upperKey);
            var modelSpeed = ShooterModel.flywheelSpeedForDistance(distance);
            return new ShootingParameters(modelSpeed, upper.trajectoryAngle, upper.timeOfFlight);
        }

        if (upperKey == null) {
            var lower = lookupTable.get(lowerKey);
            var modelSpeed = ShooterModel.flywheelSpeedForDistance(distance);
            return new ShootingParameters(modelSpeed, lower.trajectoryAngle, lower.timeOfFlight);
        }

        double lowerMeters = lowerKey.in(Meters);
        double upperMeters = upperKey.in(Meters);
        double ratio = (distance.in(Meters) - lowerMeters) / (upperMeters - lowerMeters);
        ShootingParameters lower = lookupTable.get(lowerKey);
        ShootingParameters upper = lookupTable.get(upperKey);

        // Interpolate angle and ToF; shooter speed always comes from the model.
        double lowerDeg = lower.trajectoryAngle.in(Degrees);
        double upperDeg = upper.trajectoryAngle.in(Degrees);
        double interpDeg = lerp(lowerDeg, upperDeg, ratio);

        double lowerTof = lower.timeOfFlight.in(Seconds);
        double upperTof = upper.timeOfFlight.in(Seconds);
        double interpTof = lerp(lowerTof, upperTof, ratio);

        var modelSpeed = ShooterModel.flywheelSpeedForDistance(distance);
        return new ShootingParameters(modelSpeed, Degrees.of(interpDeg), Seconds.of(interpTof));
    }

    // Prefer the Distance-based API: getParameters(Distance)
    /**
     * Convenience to return the interpolated time-of-flight for a distance.
     *
     * @param distance distance as a {@link Distance}
     * @return time-of-flight as a {@link Time}
     */
    public Time getTimeOfFlight(Distance distance) {
        return getParameters(distance).timeOfFlight;
    }

    // Prefer the Distance-based API: getTimeOfFlight(Distance)

    /** Internal helper to add a row to the table. Kept package-private for easier testing. */
    private void addEntry(Distance distance, Angle angleDeg, Time tofSec) {
        // Store a ShootingParameters instance keyed by distance. The shooterSpeed field is
        // populated from the two-point ShooterModel so the LUT does not carry per-distance RPMs.
        var modelSpeed = ShooterModel.flywheelSpeedForDistance(distance);
        lookupTable.put(distance, new ShootingParameters(modelSpeed, angleDeg, tofSec));
    }

    /** Simple linear interpolation helper. */
    private double lerp(double start, double end, double ratio) {
        return start + (end - start) * ratio;
    }

    /** Returns the minimum distance present in the lookup table, or null if empty. */
    public Distance getMinDistance() {
        return lookupTable.isEmpty() ? null : lookupTable.firstKey();
    }

    /** Returns the maximum distance present in the lookup table, or null if empty. */
    public Distance getMaxDistance() {
        return lookupTable.isEmpty() ? null : lookupTable.lastKey();
    }
}
