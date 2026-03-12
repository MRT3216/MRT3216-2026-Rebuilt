package frc.robot.util.shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.constants.ShooterConstants;

/** Two-point linear shooter model used for quick RPM lookups. */
public final class ShooterModel {
    private ShooterModel() {}

    public static AngularVelocity flywheelSpeedForDistance(Distance distance) {
        double d = distance.in(Meters);
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

    /**
     * Convenience helper returning the model RPM for a distance in inches. Useful for quick lookups
     * (e.g. spreadsheets or UI code) without constructing a {@link Distance} object.
     *
     * @param inches distance in inches
     * @return predicted flywheel speed in RPM
     */

    /**
     * Convenience helper returning the model RPM for a {@link Distance} object.
     *
     * @param distance distance as a {@link Distance}
     * @return predicted flywheel speed in RPM
     */
    public static double flywheelRPMForDistance(Distance distance) {
        return flywheelSpeedForDistance(distance).in(RPM);
    }
}
