package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.ShooterConstants.TurretConstants.kD;
import static frc.robot.constants.ShooterConstants.TurretConstants.kD_sim;
import static frc.robot.constants.ShooterConstants.TurretConstants.kGearing;
import static frc.robot.constants.ShooterConstants.TurretConstants.kHardLimitMax;
import static frc.robot.constants.ShooterConstants.TurretConstants.kHardLimitMin;
import static frc.robot.constants.ShooterConstants.TurretConstants.kI;
import static frc.robot.constants.ShooterConstants.TurretConstants.kI_sim;
import static frc.robot.constants.ShooterConstants.TurretConstants.kMOI;
import static frc.robot.constants.ShooterConstants.TurretConstants.kMotorInverted;
import static frc.robot.constants.ShooterConstants.TurretConstants.kP;
import static frc.robot.constants.ShooterConstants.TurretConstants.kP_sim;
import static frc.robot.constants.ShooterConstants.TurretConstants.kSoftLimitMax;
import static frc.robot.constants.ShooterConstants.TurretConstants.kSoftLimitMin;
import static frc.robot.constants.ShooterConstants.TurretConstants.kStartingPosition;
import static frc.robot.constants.ShooterConstants.TurretConstants.kStatorCurrentLimit;
import static frc.robot.constants.TelemetryKeys.kTurretMechTelemetry;
import static frc.robot.constants.TelemetryKeys.kTurretMotorTelemetry;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
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
import yams.motorcontrollers.local.SparkWrapper;

/**
 * AdvantageKit-ready Turret Subsystem for MRT 3216.
 *
 * <p>This subsystem manages a single-NEO turret pivot using the YAMS library and RevLib. It
 * utilizes an IO-layer abstraction for full log replay capabilities, ensuring that hardware states
 * (Inputs) are separated from software commands (Outputs).
 */
public class TurretSubsystem extends SubsystemBase {
    // region Inputs & telemetry

    /**
     * Inputs for AdvantageKit recording for the turret pivot. Public fields are populated from the
     * mechanism each loop and included in logs for replay and analysis.
     */
    @AutoLog
    public static class TurretInputs {
        /** Current turret angle (degrees). */
        public Angle angle = Degrees.of(0);

        /** Target setpoint angle (degrees) if any. */
        public Angle setpoint = Degrees.of(0);

        /** Applied voltage across the motor. */
        public Voltage volts = Volts.of(0);

        /** Measured motor current draw. */
        public Current current = Amps.of(0);
    }

    private final TurretInputsAutoLogged turretInputs = new TurretInputsAutoLogged();

    // endregion

    // region Hardware & signals

    private final SparkMax pivotMotor =
            new SparkMax(RobotMap.Shooter.Turret.kMotorId, SparkMax.MotorType.kBrushless);

    // endregion

    // region Controller configuration / mechanism

    private final SmartMotorControllerConfig motorConfig;

    /** The SmartMotorController abstraction that allows for hardware/sim parity. */
    private final SmartMotorController smartMotor;

    private final Pivot turret;

    // endregion

    // region Lifecycle / periodic

    /**
     * Updates the AdvantageKit "inputs" by refreshing hardware signals. Synchronizes TalonFX signals
     * to ensure telemetry is time-aligned.
     */
    private void updateInputs() {
        turretInputs.angle = turret.getAngle();
        turretInputs.volts = smartMotor.getVoltage();
        turretInputs.current = smartMotor.getStatorCurrent();
        turretInputs.setpoint = smartMotor.getMechanismPositionSetpoint().orElse(Degrees.of(0));
        Logger.recordOutput("Shooter/Turret/PositionDegrees", turretInputs.angle.in(Degrees));
        SmartDashboard.putBoolean(
                "Mechanisms/TurretIsMoving",
                Math.abs(turretInputs.setpoint.in(Degrees) - turretInputs.angle.in(Degrees)) > 1.0);
    }

    /** Initializes the subsystem, sets signal update frequencies, and optimizes CAN utilization. */
    public TurretSubsystem() {
        // Initialize motor controller config in constructor to avoid object-escape
        motorConfig =
                new SmartMotorControllerConfig(this)
                        .withControlMode(ControlMode.CLOSED_LOOP)
                        .withClosedLoopController(kP, kI, kD) // , kMaxVelocity, kMaxAccel)
                        .withSimClosedLoopController(kP_sim, kI_sim, kD_sim)
                        .withTelemetry(kTurretMotorTelemetry, Constants.telemetryVerbosity())
                        .withClosedLoopRampRate(Seconds.of(0.5))
                        .withGearing(kGearing)
                        .withMotorInverted(kMotorInverted)
                        .withIdleMode(MotorMode.BRAKE)
                        .withVoltageCompensation(Volts.of(12))
                        .withStatorCurrentLimit(kStatorCurrentLimit);

        smartMotor = new SparkWrapper(pivotMotor, DCMotor.getNEO(1), motorConfig);

        PivotConfig turretConfig =
                new PivotConfig(smartMotor)
                        .withStartingPosition(kStartingPosition)
                        .withTelemetry(kTurretMechTelemetry, Constants.telemetryVerbosity())
                        .withMOI(kMOI)
                        .withHardLimit(kHardLimitMin, kHardLimitMax)
                        .withSoftLimits(kSoftLimitMin, kSoftLimitMax);

        turret = new Pivot(turretConfig);
    }

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

    // region Public API - queries & commands

    /**
     * Gets the current angle of the turret.
     *
     * @return The current Angle measured by the encoder.
     */
    public Angle getPosition() {
        return turretInputs.angle;
    }

    /**
     * Returns the current commanded target for the turret.
     *
     * <p>Reads the mechanism's stored setpoint (if present) and falls back to the measured position
     * when no setpoint is available.
     */
    public Angle getTarget() {
        return smartMotor.getMechanismPositionSetpoint().orElse(getPosition());
    }

    /** Convenience: long-running closed-loop hold of the current stored target (use as default). */
    public Command stopHold() {
        return setAngle(() -> getTarget()).withName("TurretStopHold");
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
        // Allow open-loop duty outputs; mechanism-level hard/soft limits are applied
        // via PivotConfig
        return turret.set(dutyCycle);
    }

    /**
     * Sets the target angle using a dynamic supplier (e.g., from a Vision subsystem).
     *
     * @param angle A supplier providing the target Angle.
     * @return A command to track the supplier's angle.
     */
    public Command setAngle(Supplier<Angle> angle) {
        return turret.setAngle(angle);
    }

    // endregion

    // region Triggers & events

    // endregion

}
