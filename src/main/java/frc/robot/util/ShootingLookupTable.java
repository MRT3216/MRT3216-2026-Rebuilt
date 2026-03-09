package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.robot.constants.ShooterLookupTables;
import java.util.Comparator;
import java.util.TreeMap;

/**
 * Unified implementation of a shooting lookup table. Loads data from centralized constants and
 * provides interpolation. Lookups return unit-aware {@link ShootingParameters} and use {@link
 * Distance} keys for type safety.
 */
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
            addEntry(row.distance, row.shooterSpeed, row.trajectoryAngle, row.timeOfFlight);
        }
    }

    /**
     * Interpolate and return shooting parameters for a given distance.
     *
     * @param distance target distance as a {@link Distance}
     * @return interpolated shooting parameters
     */
    public ShootingParameters getParameters(Distance distance) {
        if (lookupTable.containsKey(distance)) return lookupTable.get(distance);

        Distance lowerKey = lookupTable.floorKey(distance);
        Distance upperKey = lookupTable.ceilingKey(distance);

        if (lowerKey == null) return lookupTable.get(upperKey);
        if (upperKey == null) return lookupTable.get(lowerKey);

        double lowerMeters = lowerKey.in(Meters);
        double upperMeters = upperKey.in(Meters);
        double ratio = (distance.in(Meters) - lowerMeters) / (upperMeters - lowerMeters);
        ShootingParameters lower = lookupTable.get(lowerKey);
        ShootingParameters upper = lookupTable.get(upperKey);

        // Interpolate each field in appropriate units
        double lowerRps = lower.shooterSpeed.in(RotationsPerSecond);
        double upperRps = upper.shooterSpeed.in(RotationsPerSecond);
        double interpRps = lerp(lowerRps, upperRps, ratio);

        double lowerDeg = lower.trajectoryAngle.in(Degrees);
        double upperDeg = upper.trajectoryAngle.in(Degrees);
        double interpDeg = lerp(lowerDeg, upperDeg, ratio);

        double lowerTof = lower.timeOfFlight.in(Seconds);
        double upperTof = upper.timeOfFlight.in(Seconds);
        double interpTof = lerp(lowerTof, upperTof, ratio);

        return new ShootingParameters(
                RotationsPerSecond.of(interpRps), Degrees.of(interpDeg), Seconds.of(interpTof));
    }

    // (Removed primitive overloads) Prefer the Distance-based API: getParameters(Distance)

    /**
     * Convenience to return the interpolated time-of-flight for a distance.
     *
     * @param distance distance as a {@link Distance}
     * @return time-of-flight as a {@link Time}
     */
    public Time getTimeOfFlight(Distance distance) {
        return getParameters(distance).timeOfFlight;
    }

    // (Removed primitive overloads) Prefer the Distance-based API: getTimeOfFlight(Distance)

    /** Internal helper to add a row to the table. Kept package-private for easier testing. */
    private void addEntry(Distance distance, AngularVelocity speedRps, Angle angleDeg, Time tofSec) {
        lookupTable.put(distance, new ShootingParameters(speedRps, angleDeg, tofSec));
    }

    /** Simple linear interpolation helper. */
    private double lerp(double start, double end, double ratio) {
        return start + (end - start) * ratio;
    }
}
