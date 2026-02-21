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
import edu.wpi.first.wpilibj2.command.Commands;
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
 *
 * <p>Provenance & references:
 *
 * <ul>
 *   <li>YAMS (Yet Another Mechanical System) used for the Arm abstraction — credit: BroncBotz /
 *       nstrike. If you need YAMS docs or examples, check the team's resources and the YASS
 *       tutorial videos linked in project docs.
 *   <li>Command-based best-practices: Oblarg / ChiefDelphi thread; BoVLB's "FRC Tips" guide; and
 *       WPILib command-based docs. These influenced the command-factory + trigger design used here
 *       (factories on subsystems/systems, triggers for state).
 * </ul>
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

        // Initialize the mechanism commanded setpoint to a clamped starting angle to avoid
        // commanding outside the configured soft limits at startup. Use the SmartMotorController
        // setPosition API so the mechanism's internal setpoint is consistent and observable.
        double startMeasuredDeg = hood.getAngle().in(Degrees);
        double clampedStartDeg =
                Math.max(kSoftLimitMin.in(Degrees), Math.min(kSoftLimitMax.in(Degrees), startMeasuredDeg));
        smartMotor.setPosition(Degrees.of(clampedStartDeg));
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

    /**
     * Gets the current measured hood angle.
     *
     * @return the current hood Angle measured by the mechanism/encoder
     */
    public Angle getPosition() {
        return inputs.angle;
    }

    /**
     * Returns the current commanded target for the hood.
     *
     * <p>Reads the mechanism's stored setpoint (if present) and falls back to the measured position
     * when no setpoint is available.
     */
    public Angle getTarget() {
        return smartMotor.getMechanismPositionSetpoint().orElse(getPosition());
    }

    // Note: explicit tracked `currentTarget` removed in favor of the mechanism's own setpoint.
    // Use `setAngle(...)` factories to command the mechanism (they will clamp to soft limits).

    @Override
    public void simulationPeriodic() {
        hood.simIterate();
    }

    // endregion

    // region Public API - queries & commands

    /* ----- Positioning command factories (grouped) ----- */
    /**
     * Creates a command that sets the hood to a fixed angle.
     *
     * <p>The requested angle will be clamped to the configured soft limits before being commanded.
     *
     * @param angle the desired hood Angle
     * @return a Command which, when scheduled, will move the hood to the requested angle
     */
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

    /**
     * Creates a command that sets the hood angle from a dynamic supplier.
     *
     * <p>The supplier will be evaluated at execution-time so callers can provide live targets (for
     * example, from a vision pipeline).
     *
     * @param angle supplier that provides the desired hood Angle at runtime
     * @return a Command which will track the supplier-provided angle while active
     */
    public Command setAngle(Supplier<Angle> angle) {
        // Wrap the supplier so the Arm command evaluates the supplier at execution-time.
        return hood.setAngle(angle);
    }

    /**
     * Creates a command that runs the hood motor in open-loop at the requested duty cycle.
     *
     * <p>Mechanism-level hard/soft limits configured in {@code ArmConfig} still apply.
     *
     * @param dutyCycle output percentage in the range [-1.0, 1.0]
     * @return a Command which will drive the hood motor in open-loop while active
     */
    public Command setDutyCycle(double dutyCycle) {
        // Allow open-loop duty outputs; mechanism-level hard/soft limits are applied via ArmConfig
        return hood.set(dutyCycle);
    }

    /**
     * Immediately write a clamped position setpoint to the mechanism (SmartMotorController).
     *
     * <p>This updates the SmartMotorController's stored setpoint directly (no Command is scheduled).
     * Callers that want a Command-based movement should use {@link #setAngle(Angle)}.
     *
     * @param target the desired hood Angle; it will be clamped to configured soft limits
     */
    public void setPositionImmediate(Angle target) {
        double requestedDeg = target.in(Degrees);
        double minDeg = kSoftLimitMin.in(Degrees);
        double maxDeg = kSoftLimitMax.in(Degrees);
        double clampedDeg = Math.max(minDeg, Math.min(maxDeg, requestedDeg));
        smartMotor.setPosition(Degrees.of(clampedDeg));
    }

    /**
     * Immediately bump the current commanded hood setpoint by the provided delta.
     *
     * <p>This reads the mechanism's current stored setpoint (or measured position if none), adds the
     * provided delta, and writes the resulting setpoint via {@link #setPositionImmediate(Angle)} so
     * clamping and telemetry behaviour remain centralized.
     *
     * @param delta offset to apply to the current setpoint (positive raises the hood)
     */
    public void bumpPositionImmediate(Angle delta) {
        double prevDeg = getTarget().in(Degrees);
        double newDeg = prevDeg + delta.in(Degrees);
        // Delegate to setPositionImmediate so clamping and telemetry remain centralized.
        setPositionImmediate(Degrees.of(newDeg));
    }

    /**
     * Command factory: returns a short run-once command that bumps the hood setpoint by {@code
     * delta}. The returned command requires this subsystem.
     */
    public Command bumpBy(Angle delta) {
        return Commands.runOnce(() -> bumpPositionImmediate(delta), this).withName("Hood.bumpBy");
    }

    // endregion

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
