package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

/**
 * Intake rollers constants extracted from {@code Constants.java} so they live next to the subsystem
 * implementation under `subsystems/intake`.
 */
public final class IntakeConstants {
    private IntakeConstants() {}

    /** Grouped roller constants to keep intake-related values organized. */
    public static final class Rollers {
        private Rollers() {}

        public static final Distance kWheelDiameter = Inches.of(3.5);
        public static final Mass kWheelMass = Pounds.of(2);
        public static final double kGearReduction = 1.0;
        public static final Current kStatorCurrentLimit = Amps.of(80);
        public static final double kP = 0.5;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.15;
        public static final double kV = 0.00207;
        public static final double kA = 0.0001;

        // SIM variants (tuned for sim: reduce P and feedforward a bit)
        public static final double kS_sim = 0.12;
        public static final double kV_sim = 0.001656;
        public static final double kA_sim = 0.00008;
        public static final double kP_sim = 0.35;
        public static final double kI_sim = 0.0;
        public static final double kD_sim = 0.0;
        public static final String kIntakeRollersMotorTelemetry = "IntakeRollersMotor";
        public static final String kIntakeRollersMechTelemetry = "IntakeRollersMech";
    }

    /**
     * Pivot-specific constants grouped under `IntakeConstants.Pivot` so both intake systems live in
     * the same file next to the intake subsystems.
     */
    public static final class Pivot {
        private Pivot() {}

        public static final double kGearing = 30.0;
        public static final Distance kLength = Inches.of(11.5);
        public static final Mass kMass = Pounds.of(6.4);
        public static final boolean kMotorInverted = false;
        public static final Current kStatorCurrentLimit = Amps.of(60);
        public static final double kP = 10.0;
        public static final double kI = 0.0;
        public static final double kD = 2.0;
        public static final AngularVelocity kMaxVelocity = DegreesPerSecond.of(20.0);
        public static final AngularAcceleration kMaxAccelDegPerSec2 =
                DegreesPerSecondPerSecond.of(20.0);
        public static final double kS = 0.1;
        public static final double kV = 0.12;
        public static final double kA = 0.01;
        // SIM variants (tuned for sim)
        public static final double kS_sim = 0.05;
        public static final double kV_sim = 0.09;
        public static final double kA_sim = 0.008;
        public static final double kP_sim = 6.0;
        public static final double kI_sim = 0.0;
        public static final double kD_sim = 1.0;
        public static final Angle kHardLimitMax = Degrees.of(360);
        public static final Angle kHardLimitMin = Degrees.of(0);
        public static final Angle kSoftLimitMax = Degrees.of(350);
        public static final Angle kSoftLimitMin = Degrees.of(10);
        public static final Angle kStartingPosition = Degrees.of(0);
        public static final String kIntakeArmMotorTelemetry = "IntakeArmMotor";
        public static final String kIntakeArmMechTelemetry = "IntakeArmMech";
        /** Gearing used specifically for external encoder wiring (motor:mechanism). */
        public static final double kEncoderGearing = 1.0;

        // Arm position presets
        public static final Angle kStowedAngle = Degrees.of(140);
        public static final Angle kDeployedAngle = Degrees.of(0);
    }
}
