package frc.robot.util.lookupTables;

import java.util.TreeMap;

public class PassLookUpTable implements ShootingTable { // Implements the contract
    private final TreeMap<Double, ShootingParameters> lookupTable = new TreeMap<>();

    public PassLookUpTable() {
        // Load table from centralized constants (optimized for flat field passes)
        for (var row : frc.robot.constants.Constants.ShooterLookupTables.PASS) {
            addEntry(row[0], row[1], row[2], row[3]);
        }
    }

    @Override
    public ShootingParameters getParameters(double distance) {
        Double lower = lookupTable.floorKey(distance);
        Double upper = lookupTable.ceilingKey(distance);

        if (lower == null) return lookupTable.get(upper);
        if (upper == null) return lookupTable.get(lower);

        double ratio = (distance - lower) / (upper - lower);
        var l = lookupTable.get(lower);
        var u = lookupTable.get(upper);

        return new ShootingParameters(
                lerp(l.shooterSpeed, u.shooterSpeed, ratio),
                lerp(l.trajectoryAngle, u.trajectoryAngle, ratio),
                lerp(l.timeOfFlight, u.timeOfFlight, ratio));
    }

    @Override
    public double getTimeOfFlight(double distance) {
        return getParameters(distance).timeOfFlight;
    }

    private void addEntry(double d, double s, double a, double t) {
        lookupTable.put(d, new ShootingParameters(s, a, t));
    }

    private double lerp(double s, double e, double r) {
        return s + (e - s) * r;
    }
}
