package frc.robot.util.shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.shooter.ShooterConstants;

/** Two-point linear shooter model used for quick RPM lookups. */
public final class ShooterModel {
    private ShooterModel() {}

    /** Hub-shot RPM model (default). */
    public static AngularVelocity flywheelSpeedForDistance(Distance distance) {
        return interpolate(
                distance,
                ShooterConstants.ShooterModel.dMin,
                ShooterConstants.ShooterModel.dMax,
                ShooterConstants.ShooterModel.kRpmAtMin,
                ShooterConstants.ShooterModel.kRpmAtMax);
    }

    /** Pass-shot RPM model — higher RPMs for a high-arc lob over the hub. */
    public static AngularVelocity passFlywheelSpeedForDistance(Distance distance) {
        return interpolate(
                distance,
                ShooterConstants.PassModel.dMin,
                ShooterConstants.PassModel.dMax,
                ShooterConstants.PassModel.kRpmAtMin,
                ShooterConstants.PassModel.kRpmAtMax);
    }

    /**
     * Convenience helper returning the model RPM as a raw double. Useful for calibration workflows
     * (e.g. filling in LUT hood angles at a known distance).
     *
     * @param distance distance as a {@link Distance}
     * @return predicted flywheel speed in RPM
     */
    public static double flywheelRPMForDistance(Distance distance) {
        return flywheelSpeedForDistance(distance).in(RPM);
    }

    /** Two-point linear interpolation with clamping at the endpoints. */
    private static AngularVelocity interpolate(
            Distance distance,
            Distance dMinMeasure,
            Distance dMaxMeasure,
            AngularVelocity rpmAtMinMeasure,
            AngularVelocity rpmAtMaxMeasure) {
        double d = distance.in(Meters);
        double dMin = dMinMeasure.in(Meters);
        double dMax = dMaxMeasure.in(Meters);
        double rpmAtMin = rpmAtMinMeasure.in(RPM);
        double rpmAtMax = rpmAtMaxMeasure.in(RPM);

        if (d <= dMin) return RPM.of(rpmAtMin);
        if (d >= dMax) return RPM.of(rpmAtMax);

        double t = (d - dMin) / (dMax - dMin);
        double interpRpm = rpmAtMin + (rpmAtMax - rpmAtMin) * t;
        return RPM.of(interpRpm);
    }
}
