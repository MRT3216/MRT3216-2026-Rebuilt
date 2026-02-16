package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.IntakePivotConstants;
import frc.robot.constants.RobotMap;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

/**
 * AdvantageKit-ready Intake Pivot Subsystem for MRT 3216.
 *
 * <p>This subsystem manages a single-Kraken intake arm pivot using the YAMS library and Phoenix 6.
 * It utilizes an IO-layer abstraction for full log replay capabilities, ensuring that hardware
 * states (Inputs) are separated from software commands (Outputs).
 */
public class IntakePivotSubsystem extends SubsystemBase {

    /**
     * IO inputs for the Intake Pivot. AutoLogged to provide synchronized data for AdvantageScope and
     * log replay.
     */
    @AutoLog
    public static class IntakePivotInputs {
        /** Actual angle of the intake arm. */
        public Angle angle = Degrees.of(0);
        /** Current target angle requested from the motor controller. */
        public Angle setpoint = Degrees.of(0);
        /** Applied voltage across the motor. */
        public Voltage volts = Volts.of(0);
        /** Stator current draw of the motor. */
        public Current current = Amps.of(0);
    }

    private final IntakePivotInputsAutoLogged intakePivotInputs = new IntakePivotInputsAutoLogged();

    /* Hardware controllers (left master, right follower) */
    private final SparkFlex leftPivotMotor =
            new SparkFlex(RobotMap.Intake.Pivot.kLeftMotorId, SparkFlex.MotorType.kBrushless);
    private final SparkFlex rightPivotMotor =
            new SparkFlex(RobotMap.Intake.Pivot.kRightMotorId, SparkFlex.MotorType.kBrushless);
    /* Configuration for the Smart Motor Controller (SMC) */
    private final SmartMotorControllerConfig motorConfig;

    /** The SmartMotorController abstraction that allows for hardware/sim parity. */
    private final SmartMotorController smartMotor;

    /* High-level mechanism configuration */
    private final ArmConfig intakePivotConfig;

    private final Arm intakePivot;

    /**
     * Updates the AdvantageKit "inputs" by refreshing hardware signals. Synchronizes TalonFX signals
     * to ensure telemetry is time-aligned.
     */
    private void updateInputs() {
        intakePivotInputs.angle = intakePivot.getAngle();
        intakePivotInputs.volts = smartMotor.getVoltage();
        intakePivotInputs.current = smartMotor.getStatorCurrent();

        // Sets the setpoint input based on the current SMC state
        intakePivotInputs.setpoint = smartMotor.getMechanismPositionSetpoint().orElse(Degrees.of(0));
    }

    /** Initializes the subsystem, sets signal update frequencies, and optimizes CAN utilization. */
    public IntakePivotSubsystem() {
        // Initialize motor controller config in constructor to avoid object-escape
        motorConfig =
                new SmartMotorControllerConfig(this)
                        .withControlMode(ControlMode.CLOSED_LOOP)
                        // Feedback Constants (PID Constants)
                        .withClosedLoopController(
                                IntakePivotConstants.kP, IntakePivotConstants.kI, IntakePivotConstants.kD)
                        .withSimClosedLoopController(
                                IntakePivotConstants.kP, IntakePivotConstants.kI, IntakePivotConstants.kD)
                        // Feedforward Constants
                        .withFeedforward(
                                new ArmFeedforward(
                                        IntakePivotConstants.kS, IntakePivotConstants.kV, IntakePivotConstants.kA))
                        .withSimFeedforward(
                                new ArmFeedforward(
                                        IntakePivotConstants.kS, IntakePivotConstants.kV, IntakePivotConstants.kA))
                        // Telemetry
                        .withTelemetry(IntakePivotConstants.kMotorTelemetry, TelemetryVerbosity.HIGH)
                        .withGearing(IntakePivotConstants.kGearing)
                        .withMotorInverted(IntakePivotConstants.kMotorInverted)
                        .withIdleMode(MotorMode.BRAKE)
                        .withStatorCurrentLimit(IntakePivotConstants.kStatorCurrentLimit)
                        // Configure the REV ThroughBore absolute encoder plugged into the right pivot motor
                        .withExternalEncoder(rightPivotMotor.getAbsoluteEncoder())
                        .withExternalEncoderInverted(false)
                        .withExternalEncoderGearing(
                                new MechanismGearing(
                                        GearBox.fromReductionStages(IntakePivotConstants.kEncoderGearing)))
                        .withUseExternalFeedbackEncoder(true)
                        .withExternalEncoderZeroOffset(IntakePivotConstants.kStartingPosition)
                        .withFollowers(Pair.of(rightPivotMotor, true));

        smartMotor = new SparkWrapper(leftPivotMotor, DCMotor.getNeoVortex(1), motorConfig);

        intakePivotConfig =
                new ArmConfig(smartMotor)
                        .withMass(IntakePivotConstants.kMass)
                        .withLength(IntakePivotConstants.kLength)
                        .withTelemetry(IntakePivotConstants.kMechTelemetry, TelemetryVerbosity.HIGH)
                        .withHardLimit(IntakePivotConstants.kHardLimitMin, IntakePivotConstants.kHardLimitMax)
                        .withSoftLimits(IntakePivotConstants.kSoftLimitMin, IntakePivotConstants.kSoftLimitMax);

        intakePivot = new Arm(intakePivotConfig);

        // No Phoenix status signals to configure for SparkFlex here.
    }

    /**
     * Gets the current angle of the intake arm.
     *
     * @return The current Angle measured by the encoder.
     */
    public Angle getPosition() {
        return intakePivotInputs.angle;
    }

    /**
     * Sets the target angle for the intake arm.
     *
     * @param angle The target Angle.
     * @return A command to set and maintain the requested angle.
     */
    public Command setAngle(Angle angle) {
        // Enforce configured soft limits before commanding the mechanism
        double requestedDeg = angle.in(Degrees);
        double minDeg = IntakePivotConstants.kSoftLimitMin.in(Degrees);
        double maxDeg = IntakePivotConstants.kSoftLimitMax.in(Degrees);
        double clampedDeg = Math.max(minDeg, Math.min(maxDeg, requestedDeg));
        Angle clamped = Degrees.of(clampedDeg);
        // If requested setpoint was outside soft limits, it was clamped to the allowed range.
        return intakePivot.setAngle(clamped);
    }

    /**
     * Sets the duty cycle (percent output) for the intake arm.
     *
     * @param dutyCycle The output percentage (-1.0 to 1.0).
     * @return A command to run the intake arm at the specified duty cycle.
     */
    public Command setDutyCycle(double dutyCycle) {
        // Allow open-loop duty outputs; mechanism-level hard/soft limits are applied via ArmConfig
        return intakePivot.set(dutyCycle);
    }

    @Override
    public void simulationPeriodic() {
        intakePivot.simIterate();
    }

    @Override
    public void periodic() {
        updateInputs();
        Logger.processInputs("Intake/Pivot", intakePivotInputs);
        intakePivot.updateTelemetry();
    }
}
