package frc.robot.util;

import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.constants.Constants;

/** Syncs Constants.tuningMode with NetworkTables for dashboard control. */
public class TuningModeSync {
    private static final String NT_KEY = "/Tuning/tuningMode";
    private static boolean lastValue = Constants.tuningMode;

    public static void updateFromNetworkTables() {
        boolean ntValue =
                NetworkTableInstance.getDefault()
                        .getTable("Tuning")
                        .getEntry("tuningMode")
                        .getBoolean(Constants.tuningMode);
        if (ntValue != Constants.tuningMode) {
            Constants.tuningMode = ntValue;
        }
        lastValue = ntValue;
    }

    public static void publishToNetworkTables() {
        NetworkTableInstance.getDefault()
                .getTable("Tuning")
                .getEntry("tuningMode")
                .setBoolean(Constants.tuningMode);
    }

    /** Call this periodically (e.g. from robotPeriodic) to keep in sync. */
    public static void periodic() {
        updateFromNetworkTables();
        publishToNetworkTables();
    }
}
