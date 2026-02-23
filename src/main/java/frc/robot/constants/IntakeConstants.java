package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
        // Mechanical
        public static final Distance kWheelDiameter = Inches.of(3.5);
        public static final Mass kWheelMass = Pounds.of(2);
        public static final double kGearReduction = 1.0;

        // Electrical / limits
        public static final Current kStatorCurrentLimit = Amps.of(80);

        // PID (velocity loop) - zeroed for tuning
        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        // Feedforward - zeroed for tuning
        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;

        // Simulation overrides (zeroed for tuning)
        public static final double kS_sim = 0.0;
        public static final double kV_sim = 0.0;
        public static final double kA_sim = 0.0;
        public static final double kP_sim = 0.0;
        public static final double kI_sim = 0.0;
        public static final double kD_sim = 0.0;

        // Telemetry keys
        public static final String kIntakeRollersMotorTelemetry = "IntakeRollersMotor";
        public static final String kIntakeRollersMechTelemetry = "IntakeRollersMech";

        /** Returns a preconfigured SimpleMotorFeedforward for the intake rollers. */
        public static SimpleMotorFeedforward motorFeedforward() {
            return new SimpleMotorFeedforward(kS, kV, kA);
        }

        /** Simulation variant of the intake rollers feedforward. */
        public static SimpleMotorFeedforward motorFeedforwardSim() {
            return new SimpleMotorFeedforward(kS_sim, kV_sim, kA_sim);
        }
    }

    /**
     * Pivot-specific constants grouped under `IntakeConstants.Pivot` so both intake systems live in
     * the same file next to the intake subsystems.
     */
    public static final class Pivot {
        private Pivot() {}
        // Mechanical
        public static final double kGearing = 30.0;
        public static final Distance kLength = Inches.of(13);
        public static final Mass kMass = Pounds.of(6.4);

        // Motor wiring
        public static final boolean kMotorInverted = false;
        public static final Current kStatorCurrentLimit = Amps.of(60);

        // PID / motion limits
        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final AngularVelocity kMaxVelocity = DegreesPerSecond.of(20.0);
        public static final AngularAcceleration kMaxAccelDegPerSec2 =
                DegreesPerSecondPerSecond.of(20.0);

        // Feedforward - zeroed for tuning
        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;

        /**
         * Gravity/feedforward constant used by arm-style feedforward (ArmFeedforward). Default 0.0
         * until tuned on robot; units: volts-equivalent gravity term.
         */
        public static final double kG = 0.0;

        // Simulation overrides (zeroed for tuning)
        public static final double kS_sim = 0.0;
        public static final double kV_sim = 0.0;
        public static final double kA_sim = 0.0;
        public static final double kP_sim = 0.0;
        public static final double kI_sim = 0.0;
        public static final double kD_sim = 0.0;
        /** Simulation gravity/feedforward term for ArmFeedforward in sim. */
        public static final double kG_sim = 0.0;

        // Limits / presets
        public static final Angle kHardLimitMax = Degrees.of(360);
        public static final Angle kHardLimitMin = Degrees.of(0);
        public static final Angle kSoftLimitMax = Degrees.of(350);
        public static final Angle kSoftLimitMin = Degrees.of(10);
        public static final Angle kStartingPosition = Degrees.of(0);

        // Telemetry keys
        public static final String kIntakeArmMotorTelemetry = "IntakeArmMotor";
        public static final String kIntakeArmMechTelemetry = "IntakeArmMech";

        /** Gearing used specifically for external encoder wiring (motor:mechanism). */
        public static final double kEncoderGearing = 1.0;

        // Arm position presets
        public static final Angle kStowedAngle = Degrees.of(140);
        public static final Angle kDeployedAngle = Degrees.of(0);

        /** Returns a preconfigured ArmFeedforward for the intake pivot (ks, kg, kv). */
        public static ArmFeedforward armFeedforward() {
            return new ArmFeedforward(kS, kG, kV);
        }

        /** Simulation variant for the intake pivot arm feedforward. */
        public static ArmFeedforward armFeedforwardSim() {
            return new ArmFeedforward(kS_sim, kG_sim, kV_sim);
        }
    }
}
