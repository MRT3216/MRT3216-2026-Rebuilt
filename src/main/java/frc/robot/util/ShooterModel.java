package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.constants.ShooterConstants;

/**
 * Tiny two-point linear shooter model: linearly interpolate flywheel RPM between two anchor
 * distances.
 *
 * <p>Returns an {@link AngularVelocity} (unit-aware). Distances outside the anchor range are
 * clamped to the nearest anchor speed. The model parameters are stored in {@link
 * frc.robot.constants.ShooterConstants.ShooterModel} and are expressed in meters and RPM.
 */
public final class ShooterModel {
    private ShooterModel() {}

    public static AngularVelocity flywheelSpeedForDistance(Distance distance) {
        Distance dmin = ShooterConstants.ShooterModel.dMin;
        Distance dmax = ShooterConstants.ShooterModel.dMax;
        // Anchors are stored as AngularVelocity in RPM units; extract numeric RPM for interpolation
        double rminRpm = ShooterConstants.ShooterModel.kRpmAtMin.in(RPM);
        double rmaxRpm = ShooterConstants.ShooterModel.kRpmAtMax.in(RPM);

        // Allow live tuning via NetworkTables. If the NT entries do not exist, the
        // getDouble calls return the provided default values above.
        NetworkTable nt = NetworkTableInstance.getDefault().getTable("ShooterModel");
        rminRpm = nt.getEntry("kRpmAtMin").getDouble(rminRpm);
        rmaxRpm = nt.getEntry("kRpmAtMax").getDouble(rmaxRpm);

        double dminMeters = dmin.in(Meters);
        double dmaxMeters = dmax.in(Meters);
        double distanceMeters = distance.in(Meters);

        if (Double.isNaN(distanceMeters) || distanceMeters <= dminMeters) {
            return RPM.of(rminRpm);
        }
        if (distanceMeters >= dmaxMeters) {
            return RPM.of(rmaxRpm);
        }

        double t = (distanceMeters - dminMeters) / (dmaxMeters - dminMeters);
        double rpm = rminRpm + (rmaxRpm - rminRpm) * t;
        return RPM.of(rpm);
    }
}
