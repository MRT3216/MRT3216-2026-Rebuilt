package frc.robot.util.lookupTables;

import java.util.TreeMap;

/**
 * Lookup table for high-goal Hub scoring. Optimized for steep entry angles and high-velocity shots.
 */
public class HubLookUpTable implements ShootingTable {

    private final TreeMap<Double, ShootingParameters> lookupTable = new TreeMap<>();

    public HubLookUpTable() {
        initializeLookupTable();
    }

    /**
     * Initialize with Hub-specific data points. Distance (m), Speed (RPS), Angle (°), Time of Flight
     * (s)
     */
    private void initializeLookupTable() {
        // Load table from centralized constants
        for (var row : frc.robot.constants.Constants.ShooterLookupTables.HUB) {
            addEntry(row[0], row[1], row[2], row[3]);
        }
    }

    @Override
    public ShootingParameters getParameters(double distance) {
        // Exact match check
        if (lookupTable.containsKey(distance)) {
            return lookupTable.get(distance);
        }

        Double lowerKey = lookupTable.floorKey(distance);
        Double upperKey = lookupTable.ceilingKey(distance);

        // Handle out-of-bounds
        if (lowerKey == null) return lookupTable.get(upperKey);
        if (upperKey == null) return lookupTable.get(lowerKey);

        // Linear Interpolation
        double ratio = (distance - lowerKey) / (upperKey - lowerKey);
        ShootingParameters lower = lookupTable.get(lowerKey);
        ShootingParameters upper = lookupTable.get(upperKey);

        return new ShootingParameters(
                lerp(lower.shooterSpeed, upper.shooterSpeed, ratio),
                lerp(lower.trajectoryAngle, upper.trajectoryAngle, ratio),
                lerp(lower.timeOfFlight, upper.timeOfFlight, ratio));
    }

    @Override
    public double getTimeOfFlight(double distance) {
        return getParameters(distance).timeOfFlight;
    }

    private void addEntry(double distance, double speed, double angle, double tof) {
        lookupTable.put(distance, new ShootingParameters(speed, angle, tof));
    }

    private double lerp(double start, double end, double ratio) {
        return start + (end - start) * ratio;
    }
}
