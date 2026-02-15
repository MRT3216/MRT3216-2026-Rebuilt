package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.TurretConstants;
import frc.robot.constants.RobotMap;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

/** Subsystem responsible for turret rotation and hood control. */
/**
 * AdvantageKit-ready Turret Subsystem for MRT 3216.
 *
 * <p>This subsystem manages a single-Kraken turret pivot using the YAMS library and Phoenix 6. It
 * utilizes an IO-layer abstraction for full log replay capabilities, ensuring that hardware states
 * (Inputs) are separated from software commands (Outputs).
 */
public class TurretSubsystem extends SubsystemBase {

    /**
     * Inputs for AdvantageKit recording for the turret pivot. Public fields are populated from the
     * mechanism each loop and included in logs for replay and analysis.
     */
    /**
     * Inputs for AdvantageKit recording for the turret pivot. Public fields are populated from the
     * mechanism each loop and included in logs for replay and analysis.
     */
    @AutoLog
    public static class TurretInputs {
        /** Current turret angle (degrees). */
        /** Current turret angle (degrees). */
        public Angle angle = Degrees.of(0);

        /** Target setpoint angle (degrees) if any. */

        /** Target setpoint angle (degrees) if any. */
        public Angle setpoint = Degrees.of(0);

        /** Measured motor voltage. */

        /** Measured motor voltage. */
        /** Applied voltage across the motor. */
        public Voltage volts = Volts.of(0);

        /** Measured motor current draw. */

        /** Measured motor current draw. */
        public Current current = Amps.of(0);
    }

    private final TurretInputsAutoLogged turretInputs = new TurretInputsAutoLogged();

    private TalonFX pivotMotor = new TalonFX(RobotMap.Shooter.Turret.kMotorId);
    private final StatusSignal<Angle> positionSignal = pivotMotor.getPosition();
    private final StatusSignal<Double> referenceSignal = pivotMotor.getClosedLoopReference();

    private final SmartMotorControllerConfig motorConfig;

    /** The SmartMotorController abstraction that allows for hardware/sim parity. */
    private final SmartMotorController smartMotor;

    /* High-level mechanism configuration */
    private final PivotConfig turretConfig;

    private final Pivot turret;

    /**
     * Updates the AdvantageKit "inputs" by refreshing hardware signals. Synchronizes TalonFX signals
     * to ensure telemetry is time-aligned.
     */
    private void updateInputs() {
        // Refresh all Phoenix 6 signals at once to minimize CAN latency jitter
        BaseStatusSignal.refreshAll(positionSignal, referenceSignal);

        turretInputs.angle = turret.getAngle();
        turretInputs.volts = smartMotor.getVoltage();
        turretInputs.current = smartMotor.getStatorCurrent();

        // Sets the setpoint input based on the current SMC state
        turretInputs.setpoint = smartMotor.getMechanismPositionSetpoint().orElse(Degrees.of(0));
    }

    /** Initializes the subsystem, sets signal update frequencies, and optimizes CAN utilization. */
    public TurretSubsystem() {
        // Initialize motor controller config in constructor to avoid object-escape
        motorConfig =
                new SmartMotorControllerConfig(this)
                        .withControlMode(ControlMode.CLOSED_LOOP)
                        // Feedback Constants (PID Constants)
                        .withClosedLoopController(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD)
                        .withSimClosedLoopController(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD)
                        // Feedforward Constants
                        .withFeedforward(
                                new SimpleMotorFeedforward(
                                        TurretConstants.kS, TurretConstants.kV, TurretConstants.kA))
                        .withSimFeedforward(
                                new SimpleMotorFeedforward(
                                        TurretConstants.kS, TurretConstants.kV, TurretConstants.kA))
                        // Telemetry
                        .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH)
                        .withGearing(TurretConstants.kGearing)
                        .withMotorInverted(TurretConstants.kMotorInverted)
                        .withIdleMode(MotorMode.BRAKE)
                        .withStatorCurrentLimit(TurretConstants.kStatorCurrentLimit);

        smartMotor = new TalonFXWrapper(pivotMotor, DCMotor.getKrakenX60Foc(1), motorConfig);

        turretConfig =
                new PivotConfig(smartMotor)
                        .withMOI(TurretConstants.kMOI)
                        .withTelemetry("TurretMech", TelemetryVerbosity.HIGH);

        turret = new Pivot(turretConfig);

        // High-frequency updates for PID tuning
        BaseStatusSignal.setUpdateFrequencyForAll((int) 50.0, positionSignal, referenceSignal);

        // Optimization: Disable unused signals to conserve CAN bus bandwidth
        pivotMotor.getVelocity().setUpdateFrequency(0);
    }

    /**
     * Gets the current angle of the turret.
     *
     * @return The current Angle measured by the encoder.
     */
    public Angle getPosition() {
        return turretInputs.angle;
    }

    /**
     * Sets the target angle for the turret.
     *
     * @param angle The target Angle.
     * @return A command to set and maintain the requested angle.
     */
    public Command setAngle(Angle angle) {
        return turret.setAngle(angle);
    }

    /**
     * Sets the duty cycle (percent output) for the turret.
     *
     * @param dutyCycle The output percentage (-1.0 to 1.0).
     * @return A command to run the turret at the specified duty cycle.
     */
    public Command setDutyCycle(double dutyCycle) {
        return turret.set(dutyCycle);
    }

    /**
     * Sets the target angle using a dynamic supplier (e.g., from a Vision subsystem).
     *
     * @param angle A supplier providing the target Angle.
     * @return A command to track the supplier's angle.
     */
    public Command setAngle(Supplier<Angle> angle) {
        return turret.setAngle(
                () -> {
                    Logger.recordOutput("Shooter/Turret/Setpoint", angle.get());
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
        return turret.set(
                () -> {
                    Logger.recordOutput("Shooter/Turret/DutyCycle", dutyCycle.get());
                    return dutyCycle.get();
                });
    }

    /** Advance the turret simulation model by one simulation tick. */
    /** Advance the turret simulation model by one simulation tick. */
    @Override
    public void simulationPeriodic() {
        turret.simIterate();
    }

    @Override
    public void periodic() {
        updateInputs();
        Logger.processInputs("Shooter/Turret", turretInputs);
        turret.updateTelemetry();
    }
}
