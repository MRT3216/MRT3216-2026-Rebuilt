package frc.robot.util.shooter;

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

        double lowerDeg = lower.trajectoryAngle.in(Degrees);
        double upperDeg = upper.trajectoryAngle.in(Degrees);
        double interpDeg = lerp(lowerDeg, upperDeg, ratio);

        double lowerTof = lower.timeOfFlight.in(Seconds);
        double upperTof = upper.timeOfFlight.in(Seconds);
        double interpTof = lerp(lowerTof, upperTof, ratio);

        var modelSpeed = ShooterModel.flywheelSpeedForDistance(distance);
        return new ShootingParameters(modelSpeed, Degrees.of(interpDeg), Seconds.of(interpTof));
    }

    public Time getTimeOfFlight(Distance distance) {
        return getParameters(distance).timeOfFlight;
    }

    private void addEntry(Distance distance, Angle angleDeg, Time tofSec) {
        var modelSpeed = ShooterModel.flywheelSpeedForDistance(distance);
        lookupTable.put(distance, new ShootingParameters(modelSpeed, angleDeg, tofSec));
    }

    private double lerp(double start, double end, double ratio) {
        return start + (end - start) * ratio;
    }

    public Distance getMinDistance() {
        return lookupTable.isEmpty() ? null : lookupTable.firstKey();
    }

    public Distance getMaxDistance() {
        return lookupTable.isEmpty() ? null : lookupTable.lastKey();
    }
}
