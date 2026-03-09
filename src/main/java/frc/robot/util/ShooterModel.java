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
     * @return predicted flywheel angular velocity (RPM units)
     */
    public static AngularVelocity flywheelSpeedForDistance(Distance distance) {
        double d = distance.in(Meters);
        double dMin = ShooterConstants.ShooterModel.dMin.in(Meters);
        double dMax = ShooterConstants.ShooterModel.dMax.in(Meters);

        if (d <= dMin) return ShooterConstants.ShooterModel.kRpmAtMin;
        if (d >= dMax) return ShooterConstants.ShooterModel.kRpmAtMax;

        double t = (d - dMin) / (dMax - dMin);

        double rpmMin = ShooterConstants.ShooterModel.kRpmAtMin.in(RPM);
        double rpmMax = ShooterConstants.ShooterModel.kRpmAtMax.in(RPM);
        double rpm = rpmMin + (rpmMax - rpmMin) * t;

        return RPM.of(rpm);
    }
}
