package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

/** Intake constants (rollers and pivot). Tuned by the team. */
public final class IntakeConstants {
    private IntakeConstants() {}

    // -------------------------------------------------------------------------
    // Rollers (velocity)
    // -------------------------------------------------------------------------

    public static final class Rollers {
        private Rollers() {}

        // Mechanical
        public static final Distance kWheelDiameter = Inches.of(1);
        public static final Mass kWheelMass = Pounds.of(0.5);
        public static final double kGearReduction = 2.0;

        // Motor wiring
        public static final boolean kMotorInverted = true;
        public static final Current kStatorCurrentLimit = Amps.of(40);

        // PID
        public static final double kP = 0.5;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        // Feedforward
        public static final double kS = 0.39;
        public static final double kV = 0.24;
        public static final double kA = 0.0;

        // Simulation overrides
        public static final double kP_sim = 0.01;
        public static final double kI_sim = 0.0;
        public static final double kD_sim = 0.0;
        public static final double kS_sim = 0.0;
        public static final double kV_sim = 0.2477;
        public static final double kA_sim = 0.0;

        /** Returns a preconfigured feedforward for the intake rollers. */
        public static SimpleMotorFeedforward motorFeedforward() {
            return new SimpleMotorFeedforward(kS, kV, kA);
        }

        /** Simulation variant of the intake rollers feedforward. */
        public static SimpleMotorFeedforward motorFeedforwardSim() {
            return new SimpleMotorFeedforward(kS_sim, kV_sim, kA_sim);
        }

        // Soft limits
        public static final AngularVelocity kSoftLimitMax = RPM.of(2500.0);
        public static final AngularVelocity kSoftLimitMin = RPM.of(0.0);

        // Targets / tunables
        public static final AngularVelocity kTargetAngularVelocity = RPM.of(2000.0);
    }

    // -------------------------------------------------------------------------
    // Pivot (positional)
    // -------------------------------------------------------------------------

    public static final class Pivot {
        private Pivot() {}

        // Mechanical
        public static final double kGearing = 30.0;
        public static final Distance kLength = Inches.of(11);
        public static final Mass kMass = Pounds.of(6.4);

        // Motor wiring
        public static final boolean kMotorInverted = true;
        public static final Current kStatorCurrentLimit = Amps.of(40);

        // PID
        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        // Feedforward
        public static final double kG = 0.21;
        public static final double kS = 0.11;
        public static final double kV = 0.0;
        public static final double kA = 0.0;

        // Motion profile
        public static final AngularVelocity kMaxVelocity = DegreesPerSecond.of(90.0);
        public static final AngularAcceleration kMaxAccel = DegreesPerSecondPerSecond.of(90.0);

        // Simulation overrides
        public static final double kP_sim = 0.0;
        public static final double kI_sim = 0.0;
        public static final double kD_sim = 0.0;
        public static final double kG_sim = 0.0;
        public static final double kS_sim = 0.0;
        public static final double kV_sim = 0.0;
        public static final double kA_sim = 0.0;

        /** Returns a preconfigured ArmFeedforward for the intake pivot. */
        public static ArmFeedforward armFeedforward() {
            return new ArmFeedforward(kS, kG, kV);
        }

        /** Simulation variant of the intake pivot feedforward. */
        public static ArmFeedforward armFeedforwardSim() {
            return new ArmFeedforward(kS_sim, kG_sim, kV_sim);
        }

        // Hard limits
        public static final Angle kHardLimitMax = Degrees.of(360);
        public static final Angle kHardLimitMin = Degrees.of(0);

        // Soft limits
        public static final Angle kSoftLimitMax = Degrees.of(360);
        public static final Angle kSoftLimitMin = Degrees.of(0);

        // Presets / tunables
        public static final Angle kEncoderZeroOffset = Degrees.of(123.4); // TODO: Tune this
        public static final Angle kStowedAngle = Degrees.of(125);
        public static final Angle kDeployedAngle = Degrees.of(0);
        public static final Angle kTolerance = Degrees.of(1);
    }
}
