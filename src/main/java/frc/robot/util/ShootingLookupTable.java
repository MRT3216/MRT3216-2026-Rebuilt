package frc.robot.util;

import java.util.TreeMap;

/**
 * Unified implementation of a shooting lookup table. Loads data from centralized constants and
 * provides interpolation. Use {@link Mode} to select which dataset to load.
 *
 * <p>Centralized shooting lookup table implementation used to interpolate shooter setpoints by
 * distance.
 */
public class ShootingLookupTable {
    public enum Mode {
        HUB,
        PASS
    }

    private final TreeMap<Double, ShootingParameters> lookupTable = new TreeMap<>();

    /**
     * Create a ShootingLookupTable that loads the requested dataset from Constants.
     *
     * @param mode the dataset mode to load (HUB or PASS)
     */
    public ShootingLookupTable(Mode mode) {
        loadFromConstants(mode);
    }

    private void loadFromConstants(Mode mode) {
        double[][] data;
        switch (mode) {
            case HUB:
                data = frc.robot.constants.Constants.ShooterLookupTables.HUB;
                break;
            case PASS:
                data = frc.robot.constants.Constants.ShooterLookupTables.PASS;
                break;
            default:
                data = new double[0][];
        }

        for (var row : data) {
            addEntry(row[0], row[1], row[2], row[3]);
        }
    }

    /**
     * Interpolate and return shooting parameters for a given distance.
     *
     * @param distance distance in meters
     * @return interpolated shooting parameters
     */
    public ShootingParameters getParameters(double distance) {
        if (lookupTable.containsKey(distance)) return lookupTable.get(distance);

        Double lowerKey = lookupTable.floorKey(distance);
        Double upperKey = lookupTable.ceilingKey(distance);

        if (lowerKey == null) return lookupTable.get(upperKey);
        if (upperKey == null) return lookupTable.get(lowerKey);

        double ratio = (distance - lowerKey) / (upperKey - lowerKey);
        ShootingParameters lower = lookupTable.get(lowerKey);
        ShootingParameters upper = lookupTable.get(upperKey);

        return new ShootingParameters(
                lerp(lower.shooterSpeed, upper.shooterSpeed, ratio),
                lerp(lower.trajectoryAngle, upper.trajectoryAngle, ratio),
                lerp(lower.timeOfFlight, upper.timeOfFlight, ratio));
    }

    /**
     * Convenience to return the interpolated time-of-flight for a distance.
     *
     * @param distance distance in meters
     * @return time-of-flight in seconds
     */
    public double getTimeOfFlight(double distance) {
        return getParameters(distance).timeOfFlight;
    }

    /** Internal helper to add a row to the table. Kept package-private for easier testing. */
    private void addEntry(double distance, double speed, double angle, double tof) {
        lookupTable.put(distance, new ShootingParameters(speed, angle, tof));
    }

    /** Simple linear interpolation helper. */
    private double lerp(double start, double end, double ratio) {
        return start + (end - start) * ratio;
    }
}
