package frc.robot.util.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.robot.constants.ShooterLookupTables;
import java.util.Comparator;
import java.util.Optional;
import java.util.TreeMap;

/** Shooting lookup table: loads embedded constants and provides unit-aware interpolation. */
public class ShootingLookupTable {
    public enum Mode {
        HUB,
        PASS
    }

    private final TreeMap<Distance, ShootingParameters> lookupTable =
            new TreeMap<>(Comparator.comparingDouble(d -> d.in(Meters)));

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

    public ShootingParameters getParameters(Distance distance) {
        Distance lowerKey = lookupTable.floorKey(distance);
        Distance upperKey = lookupTable.ceilingKey(distance);

        if (lowerKey == null && upperKey == null) {
            return new ShootingParameters(Degrees.of(Double.NaN), Seconds.of(Double.NaN));
        }

        if (lowerKey == null) {
            var upper = lookupTable.get(upperKey);
            return new ShootingParameters(upper.trajectoryAngle(), upper.timeOfFlight());
        }

        if (upperKey == null) {
            var lower = lookupTable.get(lowerKey);
            return new ShootingParameters(lower.trajectoryAngle(), lower.timeOfFlight());
        }

        double lowerMeters = lowerKey.in(Meters);
        double upperMeters = upperKey.in(Meters);
        double ratio = (distance.in(Meters) - lowerMeters) / (upperMeters - lowerMeters);
        ShootingParameters lower = lookupTable.get(lowerKey);
        ShootingParameters upper = lookupTable.get(upperKey);

        double interpDeg =
                lerp(lower.trajectoryAngle().in(Degrees), upper.trajectoryAngle().in(Degrees), ratio);
        double interpTof =
                lerp(lower.timeOfFlight().in(Seconds), upper.timeOfFlight().in(Seconds), ratio);

        return new ShootingParameters(Degrees.of(interpDeg), Seconds.of(interpTof));
    }

    public Time getTimeOfFlight(Distance distance) {
        return getParameters(distance).timeOfFlight();
    }

    private void addEntry(Distance distance, Angle angleDeg, Time tofSec) {
        lookupTable.put(distance, new ShootingParameters(angleDeg, tofSec));
    }

    private double lerp(double start, double end, double ratio) {
        return start + (end - start) * ratio;
    }

    public Optional<Distance> getMinDistance() {
        return lookupTable.isEmpty() ? Optional.empty() : Optional.of(lookupTable.firstKey());
    }

    public Optional<Distance> getMaxDistance() {
        return lookupTable.isEmpty() ? Optional.empty() : Optional.of(lookupTable.lastKey());
    }

    /**
     * Returns the minimum time-of-flight (seconds) across all entries — the TOF at the closest
     * distance in the table. Used by HubShiftUtil to compute shift fudge factors.
     */
    public double getMinTimeOfFlight() {
        if (lookupTable.isEmpty()) return 0.0;
        return lookupTable.get(lookupTable.firstKey()).timeOfFlight().in(Seconds);
    }

    /**
     * Returns the maximum time-of-flight (seconds) across all entries — the TOF at the farthest
     * distance in the table. Used by HubShiftUtil to compute shift fudge factors.
     */
    public double getMaxTimeOfFlight() {
        if (lookupTable.isEmpty()) return 0.0;
        return lookupTable.get(lookupTable.lastKey()).timeOfFlight().in(Seconds);
    }
}
