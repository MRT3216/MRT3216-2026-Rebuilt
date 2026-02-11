package frc.robot.util.lookupTables;

public interface ShootingTable {
    // Adding 'public' to the class and every field
    public static class ShootingParameters {
        public final double shooterSpeed;
        public final double trajectoryAngle;
        public final double timeOfFlight;

        public ShootingParameters(double shooterSpeed, double trajectoryAngle, double timeOfFlight) {
            this.shooterSpeed = shooterSpeed;
            this.trajectoryAngle = trajectoryAngle;
            this.timeOfFlight = timeOfFlight;
        }
    }

    ShootingParameters getParameters(double distance);

    double getTimeOfFlight(double distance);
}
