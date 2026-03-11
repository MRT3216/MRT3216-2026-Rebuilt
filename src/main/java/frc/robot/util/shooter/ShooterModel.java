package frc.robot.util.shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.constants.ShooterConstants;

/** Very small two-point linear shooter model. */
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
}
