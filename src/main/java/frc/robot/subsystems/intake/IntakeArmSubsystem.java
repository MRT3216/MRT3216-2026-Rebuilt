package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.IntakeArmConstants;
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
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

/**
 * AdvantageKit-ready Intake Arm Subsystem for MRT 3216.
 *
 * <p>This subsystem manages a single-Kraken intake arm pivot using the YAMS library and Phoenix 6.
 * It utilizes an IO-layer abstraction for full log replay capabilities, ensuring that hardware
 * states (Inputs) are separated from software commands (Outputs).
 */
public class IntakeArmSubsystem extends SubsystemBase {

    /**
     * IO inputs for the Intake Arm. AutoLogged to provide synchronized data for AdvantageScope and
     * log replay.
     */
    @AutoLog
    public static class IntakeArmInputs {
        /** Actual angle of the intake arm. */
        public Angle angle = Degrees.of(0);
        /** Current target angle requested from the motor controller. */
        public Angle setpoint = Degrees.of(0);
        /** Applied voltage across the motor. */
        public Voltage volts = Volts.of(0);
        /** Stator current draw of the motor. */
        public Current current = Amps.of(0);
    }

    private final IntakeArmInputsAutoLogged intakeArmInputs = new IntakeArmInputsAutoLogged();

    /* Hardware Objects */
    private final TalonFX LeftPivotMotor = new TalonFX(RobotMap.Intake.Arm.kLeftPivotMotorId);
    private final TalonFX RightPivotMotor = new TalonFX(RobotMap.Intake.Arm.kRightPivotMotorId);

    /* Phoenix 6 Status Signals (for high-frequency synchronized logging) */
    private final StatusSignal<Angle> positionSignal = LeftPivotMotor.getPosition();
    private final StatusSignal<Double> referenceSignal = LeftPivotMotor.getClosedLoopReference();
    /* Configuration for the Smart Motor Controller (SMC) */
    private final SmartMotorControllerConfig motorConfig;

    /** The SmartMotorController abstraction that allows for hardware/sim parity. */
    private final SmartMotorController smartMotor;

    /* High-level mechanism configuration */
    private final ArmConfig intakeArmConfig;

    private final Arm intakeArm;

    /**
     * Updates the AdvantageKit "inputs" by refreshing hardware signals. Synchronizes TalonFX signals
     * to ensure telemetry is time-aligned.
     */
    private void updateInputs() {
        // Refresh all Phoenix 6 signals at once to minimize CAN latency jitter
        BaseStatusSignal.refreshAll(positionSignal, referenceSignal);

        intakeArmInputs.angle = intakeArm.getAngle();
        intakeArmInputs.volts = smartMotor.getVoltage();
        intakeArmInputs.current = smartMotor.getStatorCurrent();

        // Sets the setpoint input based on the current SMC state
        intakeArmInputs.setpoint = smartMotor.getMechanismPositionSetpoint().orElse(Degrees.of(0));
    }

    /** Initializes the subsystem, sets signal update frequencies, and optimizes CAN utilization. */
    public IntakeArmSubsystem() {
        // Initialize motor controller config in constructor to avoid object-escape
        motorConfig =
                new SmartMotorControllerConfig(this)
                        .withControlMode(ControlMode.CLOSED_LOOP)
                        // Feedback Constants (PID Constants)
                        .withClosedLoopController(
                                IntakeArmConstants.kP, IntakeArmConstants.kI, IntakeArmConstants.kD)
                        .withSimClosedLoopController(
                                IntakeArmConstants.kP, IntakeArmConstants.kI, IntakeArmConstants.kD)
                        // Feedforward Constants
                        .withFeedforward(
                                new ArmFeedforward(
                                        IntakeArmConstants.kS, IntakeArmConstants.kV, IntakeArmConstants.kA))
                        .withSimFeedforward(
                                new ArmFeedforward(
                                        IntakeArmConstants.kS, IntakeArmConstants.kV, IntakeArmConstants.kA))
                        // Telemetry
                        .withTelemetry(IntakeArmConstants.kMotorTelemetry, TelemetryVerbosity.HIGH)
                        .withGearing(IntakeArmConstants.kGearing)
                        .withMotorInverted(IntakeArmConstants.kMotorInverted)
                        .withIdleMode(MotorMode.BRAKE)
                        .withStatorCurrentLimit(IntakeArmConstants.kStatorCurrentLimit)
                        .withFollowers(Pair.of(RightPivotMotor, true));

        smartMotor = new TalonFXWrapper(LeftPivotMotor, DCMotor.getKrakenX60Foc(1), motorConfig);

        intakeArmConfig =
                new ArmConfig(smartMotor)
                        .withMOI(IntakeArmConstants.kMOI)
                        .withTelemetry(IntakeArmConstants.kMechTelemetry, TelemetryVerbosity.HIGH);

        intakeArm = new Arm(intakeArmConfig);

        // High-frequency updates for PID tuning
        BaseStatusSignal.setUpdateFrequencyForAll((int) 50.0, positionSignal, referenceSignal);

        // Optimization: Disable unused signals to conserve CAN bus bandwidth
        LeftPivotMotor.getVelocity().setUpdateFrequency(0);
    }

    /**
     * Gets the current angle of the intake arm.
     *
     * @return The current Angle measured by the encoder.
     */
    public Angle getPosition() {
        return intakeArmInputs.angle;
    }

    /**
     * Sets the target angle for the intake arm.
     *
     * @param angle The target Angle.
     * @return A command to set and maintain the requested angle.
     */
    public Command setAngle(Angle angle) {
        return intakeArm.setAngle(angle);
    }

    /**
     * Sets the duty cycle (percent output) for the intake arm.
     *
     * @param dutyCycle The output percentage (-1.0 to 1.0).
     * @return A command to run the intake arm at the specified duty cycle.
     */
    public Command setDutyCycle(double dutyCycle) {
        return intakeArm.set(dutyCycle);
    }

    /**
     * Sets the target angle using a dynamic supplier (e.g., from a Vision subsystem).
     *
     * @param angle A supplier providing the target Angle.
     * @return A command to track the supplier's angle.
     */
    public Command setAngle(Supplier<Angle> angle) {
        return intakeArm.setAngle(
                () -> {
                    Logger.recordOutput("Intake/Arm/Setpoint", angle.get());
                    return angle.get();
                });
    }

    /**
     * Sets the duty cycle using a dynamic supplier.
     *
     * @param dutyCycle A supplier providing the target duty cycle.
     * @return A command to track the supplier's duty cycle.
     */
    public Command setDutyCycle(Supplier<Double> dutyCycle) {
        return intakeArm.set(
                () -> {
                    Logger.recordOutput("Intake/Arm/DutyCycle", dutyCycle.get());
                    return dutyCycle.get();
                });
    }

    @Override
    public void simulationPeriodic() {
        intakeArm.simIterate();
    }

    @Override
    public void periodic() {
        updateInputs();
        Logger.processInputs("Intake/Arm", intakeArmInputs);
        intakeArm.updateTelemetry();
    }
}
