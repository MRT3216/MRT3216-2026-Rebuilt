package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.IntakeConstants.Pivot.*;

import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.RobotMap;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.local.SparkWrapper;

/**
 * AdvantageKit-ready Intake Pivot Subsystem for MRT 3216.
 *
 * <p>This subsystem manages a single-Kraken intake arm pivot using the YAMS library and Phoenix 6.
 * It utilizes an IO-layer abstraction for full log replay capabilities, ensuring that hardware
 * states (Inputs) are separated from software commands (Outputs).
 */
public class IntakePivotSubsystem extends SubsystemBase {
    // region Inputs & telemetry

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

    // endregion

    // region Hardware & controllers

    /* Hardware controllers (left master, right follower) */
    private final SparkFlex leftPivotMotor =
            new SparkFlex(RobotMap.Intake.Pivot.kLeftMotorId, SparkFlex.MotorType.kBrushless);
    private final SparkFlex rightPivotMotor =
            new SparkFlex(RobotMap.Intake.Pivot.kRightMotorId, SparkFlex.MotorType.kBrushless);

    // endregion

    // region Controller configuration / mechanism
    /* Configuration for the Smart Motor Controller (SMC) */
    private final SmartMotorControllerConfig motorConfig;

    /** The SmartMotorController abstraction that allows for hardware/sim parity. */
    private final SmartMotorController smartMotor;

    /* High-level mechanism configuration */
    private final ArmConfig intakePivotConfig;

    private final Arm intakePivot;

    // endregion

    // region Initialization helpers

    /** Initializes the subsystem, sets signal update frequencies, and optimizes CAN utilization. */
    public IntakePivotSubsystem() {
        // Initialize motor controller config in constructor to avoid object-escape
        motorConfig =
                new SmartMotorControllerConfig(this)
                        .withControlMode(ControlMode.CLOSED_LOOP)
                        // Feedback Constants (PID Constants) + motion limits for trapezoidal profiles
                        .withClosedLoopController(kP, kI, kD, kMaxVelocity, kMaxAccel)
                        .withSimClosedLoopController(kP_sim, kI_sim, kD_sim, kMaxVelocity, kMaxAccel)
                        // Use centralized intake pivot arm feedforward factory
                        .withFeedforward(armFeedforward())
                        .withSimFeedforward(armFeedforwardSim())
                        // Telemetry
                        .withTelemetry(kIntakeArmMotorTelemetry, Constants.telemetryVerbosity())
                        .withGearing(kGearing)
                        .withMotorInverted(kMotorInverted)
                        .withIdleMode(MotorMode.BRAKE)
                        // Enable REV voltage compensation (12V) so the arm's closed-loop
                        // performance stays consistent across battery voltage swings. CTRE's
                        // TalonFX controllers don't expose the same YAMS voltage-compensation
                        // integration, so we don't enable it for Phoenix-backed controllers.
                        .withVoltageCompensation(Volts.of(12))
                        .withStatorCurrentLimit(kStatorCurrentLimit)
                        // Configure the REV ThroughBore absolute encoder plugged into the right pivot motor
                        // .withExternalEncoder(rightPivotMotor.getAbsoluteEncoder())
                        // withExternalEncoderInverted(false)
                        // .withExternalEncoderGearing(
                        //         new MechanismGearing(GearBox.fromReductionStages(kEncoderGearing)))
                        .withUseExternalFeedbackEncoder(false)
                        // .withExternalEncoderZeroOffset(kEncoderOffset)
                        .withFollowers(Pair.of(rightPivotMotor, true));

        smartMotor = new SparkWrapper(leftPivotMotor, DCMotor.getNeoVortex(1), motorConfig);

        intakePivotConfig =
                new ArmConfig(smartMotor)
                        .withMass(kMass)
                        .withLength(kLength)
                        .withTelemetry(kIntakeArmMechTelemetry, Constants.telemetryVerbosity())
                        // Provide a starting position so the Arm has a known initial angle
                        .withStartingPosition(kEncoderOffset)
                        .withHardLimit(kHardLimitMin, kHardLimitMax)
                        .withSoftLimits(kSoftLimitMin, kSoftLimitMax);

        intakePivot = new Arm(intakePivotConfig);
        // Initialize the mechanism commanded setpoint to a clamped starting angle to
        // avoid commanding outside the configured soft limits at startup. Use the
        // SmartMotorController setPosition API so the mechanism's internal setpoint is
        // consistent and observable.
        double startMeasuredDeg = intakePivot.getAngle().in(Degrees);
        double clampedStartDeg =
                MathUtil.clamp(startMeasuredDeg, kSoftLimitMin.in(Degrees), kSoftLimitMax.in(Degrees));
        smartMotor.setPosition(Degrees.of(clampedStartDeg));

        // No Phoenix status signals to configure for SparkFlex here.
    }

    // endregion

    // region Lifecycle / periodic

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
    // endregion

    // region Public API (queries & commands)

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
        double minDeg = kSoftLimitMin.in(Degrees);
        double maxDeg = kSoftLimitMax.in(Degrees);
        double clampedDeg = MathUtil.clamp(requestedDeg, minDeg, maxDeg);
        Angle clamped = Degrees.of(clampedDeg);
        // If requested setpoint was outside soft limits, it was clamped to the allowed range.
        return intakePivot.setAngle(clamped);
    }

    /**
     * Supplier-backed overload for dynamic angle targets (e.g., live tuning or vision).
     *
     * @param angle supplier providing the desired Angle
     * @return a Command that follows the supplier while active
     */
    public Command setAngle(Supplier<Angle> angle) {
        return intakePivot.setAngle(angle);
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

    /**
     * Returns the current commanded target for the intake pivot.
     *
     * <p>Reads the mechanism's stored setpoint (if present) and falls back to the measured position
     * when no setpoint is available.
     */
    public Angle getTarget() {
        return smartMotor.getMechanismPositionSetpoint().orElse(getPosition());
    }

    /**
     * Immediately write a clamped position setpoint to the mechanism (YAMS Arm).
     *
     * <p>This updates the mechanism's stored setpoint directly (no Command is scheduled). It clamps
     * to the configured soft limits before writing via the Arm API so the mechanism remains the
     * single source of truth for commanded setpoints.
     *
     * @param target the desired Angle; it will be clamped to configured soft limits
     */
    public void writeSetpointImmediate(Angle target) {
        double requestedDeg = target.in(Degrees);
        double minDeg = kSoftLimitMin.in(Degrees);
        double maxDeg = kSoftLimitMax.in(Degrees);
        double clampedDeg = MathUtil.clamp(requestedDeg, minDeg, maxDeg);
        intakePivot.setMechanismPositionSetpoint(Degrees.of(clampedDeg));
    }

    // endregion

    // region Triggers & events

    // (none yet) — trigger wiring is done in RobotContainer/Systems

    // endregion
}
