package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.ShooterConstants.HoodConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
import yams.motorcontrollers.remote.TalonFXWrapper;

/**
 * Hood pivot subsystem — controls the shooting angle.
 *
 * <p>Modeled as a positional arm using the YAMS {@code Arm} abstraction and a TalonFX motor
 * controller. This subsystem exposes small command factories (wrapping the YAMS commands) and
 * maintains a tracked commanded target used by the default hold command.
 */
public class HoodSubsystem extends SubsystemBase {
    // region Inputs & telemetry
    @AutoLog
    public static class HoodInputs {
        public Angle angle = Degrees.of(0);
        public Angle setpoint = Degrees.of(0);
        public Voltage volts = Volts.of(0);
        public Current current = Amps.of(0);
    }

    private final HoodInputsAutoLogged inputs = new HoodInputsAutoLogged();
    // endregion

    // region Hardware & signals
    private final TalonFX motor = new TalonFX(RobotMap.Shooter.Hood.kMotorId);

    private final StatusSignal<Angle> positionSignal = motor.getPosition();
    private final StatusSignal<Double> referenceSignal = motor.getClosedLoopReference();
    // endregion

    // region Controller configuration / mechanism
    private final SmartMotorControllerConfig motorConfig;
    private final SmartMotorController smartMotor;
    private final ArmConfig hoodConfig;
    private final Arm hood;
    // endregion

    // region Target tracking
    /** Current commanded target for the hood; used by the default hold command. */
    private volatile Angle currentTarget;

    /* Debug / verbose logging intentionally removed from this file. Use the centralized
     * Logger.processInputs(...) calls already present in periodic() for telemetry. */
    // endregion

    public HoodSubsystem() {
        motorConfig =
                new SmartMotorControllerConfig(this)
                        .withControlMode(ControlMode.CLOSED_LOOP)
                        .withClosedLoopController(kP, kI, kD)
                        .withSimClosedLoopController(kP_sim, kI_sim, kD_sim)
                        .withFeedforward(new ArmFeedforward(kS, kV, kA))
                        .withSimFeedforward(new ArmFeedforward(kS_sim, kV_sim, kA_sim))
                        .withTelemetry(kHoodMotorTelemetry, Constants.telemetryVerbosity())
                        .withGearing(kGearing)
                        .withMotorInverted(kMotorInverted)
                        .withIdleMode(MotorMode.BRAKE)
                        .withStatorCurrentLimit(kStatorCurrentLimit);

        smartMotor = new TalonFXWrapper(motor, DCMotor.getKrakenX44Foc(1), motorConfig);

        hoodConfig =
                new ArmConfig(smartMotor)
                        .withMass(kMass)
                        .withLength(kLength)
                        .withTelemetry(kHoodMechTelemetry, Constants.telemetryVerbosity())
                        // Ensure Arm has a known starting angle for simulation and replay
                        .withStartingPosition(kStartingPosition)
                        .withHardLimit(kHardLimitMin, kHardLimitMax)
                        .withSoftLimits(kSoftLimitMin, kSoftLimitMax);

        hood = new Arm(hoodConfig);

        // Initialize the commanded target to a clamped starting angle to avoid
        // commanding outside the configured soft limits at startup.
        double startMeasuredDeg = hood.getAngle().in(Degrees);
        double clampedStartDeg =
                Math.max(kSoftLimitMin.in(Degrees), Math.min(kSoftLimitMax.in(Degrees), startMeasuredDeg));
        currentTarget = Degrees.of(clampedStartDeg);
        // Optimize CAN update frequency for the signals we use (centralized default)
        BaseStatusSignal.setUpdateFrequencyForAll(
                Constants.CommsConstants.DEFAULT_TELEMETRY_HZ, positionSignal, referenceSignal);
        motor.getVelocity().setUpdateFrequency(0);
    }

    // region Lifecycle / periodic

    private void updateInputs() {
        // Refresh Phoenix signals so logged telemetry is time-aligned with hardware.
        BaseStatusSignal.refreshAll(positionSignal, referenceSignal);

        inputs.angle = hood.getAngle();
        inputs.volts = smartMotor.getVoltage();
        inputs.current = smartMotor.getStatorCurrent();
        inputs.setpoint = smartMotor.getMechanismPositionSetpoint().orElse(Degrees.of(0));
    }

    public Angle getPosition() {
        return inputs.angle;
    }

    /** Returns the current commanded target for the hood. */
    public Angle getTarget() {
        return currentTarget;
    }

    /**
     * Set the tracked commanded target directly (clamped to soft limits).
     *
     * <p>This method does not schedule any commands; it only updates the internal tracked target that
     * the default hold command will use.
     *
     * @param target the desired hood angle; it will be clamped to configured soft limits
     */
    public void setTarget(Angle target) {
        double requestedDeg = target.in(Degrees);
        double minDeg = kSoftLimitMin.in(Degrees);
        double maxDeg = kSoftLimitMax.in(Degrees);
        double clampedDeg = Math.max(minDeg, Math.min(maxDeg, requestedDeg));
        currentTarget = Degrees.of(clampedDeg);
    }

    @Override
    public void simulationPeriodic() {
        hood.simIterate();
    }

    // endregion

    // region Public API - queries & commands

    /* ----- Positioning command factories (grouped) ----- */
    public Command setAngle(Angle angle) {
        // Clamp requested angle to configured soft limits
        double requestedDeg = angle.in(Degrees);
        double minDeg = kSoftLimitMin.in(Degrees);
        double maxDeg = kSoftLimitMax.in(Degrees);
        double clampedDeg = Math.max(minDeg, Math.min(maxDeg, requestedDeg));
        Angle clamped = Degrees.of(clampedDeg);
        // If requested setpoint was outside soft limits, it was clamped to the allowed range.
        return hood.setAngle(clamped);
    }

    public Command setAngle(Supplier<Angle> angle) {
        // Wrap the supplier so the Arm command evaluates the supplier at execution-time.
        return hood.setAngle(() -> angle.get());
    }

    public Command setDutyCycle(double dutyCycle) {
        // Allow open-loop duty outputs; mechanism-level hard/soft limits are applied via ArmConfig
        return hood.set(dutyCycle);
    }
    // endregion
    // file-based sim logging removed

    /** Trigger active when the hood is within the configured position tolerance of the setpoint. */
    public final Trigger atSetpoint =
            new Trigger(
                    () -> {
                        Angle tgt = inputs.setpoint;
                        double diff = Math.abs(getPosition().in(Degrees) - tgt.in(Degrees));
                        return diff <= kPositionTolerance.in(Degrees);
                    });

    @Override
    public void periodic() {
        updateInputs();
        Logger.processInputs("Hood", inputs);
        // (removed short-lived trace samples)

        hood.updateTelemetry();
    }
    // endregion
}
