package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.constants.ShooterConstants;

/**
 * Very small two-point linear shooter model. Interpolates flywheel RPM between two anchors
 * specified in {@link frc.robot.constants.ShooterConstants.ShooterModel} based on distance.
 */
public final class ShooterModel {
    private ShooterModel() {}

    /**
     * Return a model-predicted flywheel angular velocity for the provided distance.
     *
     * @param distance target lead distance
     * @return predicted flywheel angular velocity (RPM)
     */
    public static AngularVelocity flywheelSpeedForDistance(Distance distance) {
        double d = distance.in(Meters);

        // Always use the configured two-point anchors in ShooterConstants.ShooterModel.
        // The project decision is to base model flywheel speeds solely on those constants
        // (kRpmAtMin/kRpmAtMax) and the configured distance bounds (dMin/dMax).
        double dMin = ShooterConstants.ShooterModel.dMin.in(Meters);
        double dMax = ShooterConstants.ShooterModel.dMax.in(Meters);
        double rpmAtMin = ShooterConstants.ShooterModel.kRpmAtMin.in(RPM);
        double rpmAtMax = ShooterConstants.ShooterModel.kRpmAtMax.in(RPM);

        if (d <= dMin) return RPM.of(rpmAtMin);
        if (d >= dMax) return RPM.of(rpmAtMax);

        double t = (d - dMin) / (dMax - dMin);

        double interpRpm = rpmAtMin + (rpmAtMax - rpmAtMin) * t;
        return RPM.of(interpRpm);
    }
}
