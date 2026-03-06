package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.ShooterConstants.HoodConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
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
import frc.robot.util.PhoenixUtil;
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

    // endregion

    public HoodSubsystem() {
        motorConfig =
                new SmartMotorControllerConfig(this)
                        .withControlMode(ControlMode.CLOSED_LOOP)
                        .withClosedLoopController(kP, kI, kD, kMaxVelocity, kMaxAccel)
                        .withSimClosedLoopController(kP_sim, kI_sim, kD_sim, kMaxVelocity, kMaxAccel)
                        // Use centralized Hood feedforward factory
                        .withFeedforward(armFeedforward())
                        .withSimFeedforward(armFeedforwardSim())
                        .withTelemetry(kHoodMotorTelemetry, Constants.telemetryVerbosity())
                        .withGearing(kGearing)
                        .withMotorInverted(kMotorInverted)
                        .withIdleMode(MotorMode.BRAKE)
                        // NOTE: This subsystem uses a TalonFX (CTRE Phoenix). The Phoenix
                        // library doesn't provide a YAMS-accessible `.withVoltageCompensation(...)`
                        // method like REV SmartMotorController wrappers do, so we don't enable
                        // voltage compensation here.
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

        // Initialize the mechanism commanded setpoint to a clamped starting angle to
        // avoid commanding outside the configured soft limits at startup. Use the
        // SmartMotorController setPosition API so the mechanism's internal setpoint is
        // consistent and
        // observable.
        double startMeasuredDeg = hood.getAngle().in(Degrees);
        double clampedStartDeg =
                MathUtil.clamp(startMeasuredDeg, kSoftLimitMin.in(Degrees), kSoftLimitMax.in(Degrees));
        smartMotor.setPosition(Degrees.of(clampedStartDeg));
        // Optimize CAN update frequency for the signals we use (centralized default)
        BaseStatusSignal.setUpdateFrequencyForAll(
                Constants.CommsConstants.DEFAULT_TELEMETRY_HZ, positionSignal, referenceSignal);
        motor.getVelocity().setUpdateFrequency(0);
    }

    // region Lifecycle / periodic

    private void updateInputs() {
        // Refresh Phoenix signals so logged telemetry is time-aligned with hardware.
        PhoenixUtil.refresh(positionSignal, referenceSignal);

        // Record raw Phoenix signals (helpful for Phoenix Tuner / AdvantageScope plots)
        Logger.recordOutput("Hood/FX/PositionDegrees", positionSignal.getValue().in(Degrees));
        Logger.recordOutput(
                "Hood/FX/ReferenceDegrees", Degrees.convertFrom(referenceSignal.getValue(), Rotations));

        inputs.angle = hood.getAngle();
        inputs.volts = smartMotor.getVoltage();
        inputs.current = smartMotor.getStatorCurrent();
        inputs.setpoint = smartMotor.getMechanismPositionSetpoint().orElse(Degrees.of(0));
    }

    @Override
    public void periodic() {
        updateInputs();
        Logger.processInputs("Hood", inputs);
        hood.updateTelemetry();
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

    /**
     * /** Sets the hood to a fixed angle. The requested angle will be clamped to configured soft
     * limits before being commanded.
     */
    public Command setAngle(Angle angle) {
        double requestedDeg = angle.in(Degrees);
        double minDeg = kSoftLimitMin.in(Degrees);
        double maxDeg = kSoftLimitMax.in(Degrees);
        double clampedDeg = MathUtil.clamp(requestedDeg, minDeg, maxDeg);
        return hood.setAngle(Degrees.of(clampedDeg));
    }

    /** Supplier-backed overload for dynamic angles. */
    public Command setAngle(Supplier<Angle> angle) {
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
        // Allow open-loop duty outputs; mechanism-level hard/soft limits are applied
        // via ArmConfig
        return hood.set(dutyCycle);
    }

    /**
     * Immediately write a clamped position setpoint to the mechanism (YAMS Arm).
     *
     * <p>This updates the mechanism's stored setpoint directly (no Command is scheduled). It clamps
     * to the configured soft limits before writing via the Arm API so the mechanism remains the
     * single source of truth for commanded setpoints.
     *
     * @param target the desired hood Angle; it will be clamped to configured soft limits
     */
    private void writeSetpointImmediate(Angle target) {
        double requestedDeg = target.in(Degrees);
        double minDeg = kSoftLimitMin.in(Degrees);
        double maxDeg = kSoftLimitMax.in(Degrees);
        double clampedDeg = MathUtil.clamp(requestedDeg, minDeg, maxDeg);
        hood.setMechanismPositionSetpoint(Degrees.of(clampedDeg));
    }

    /**
     * Applies an immediate, clamped offset to the currently commanded setpoint.
     *
     * <p>The method reads the latest commanded setpoint from the SmartMotorController when available
     * (falls back to the measured position), adds {@code delta}, clamps the result, and writes it via
     * {@link #writeSetpointImmediate(Angle)} so clamping and telemetry remain centralized.
     *
     * @param delta offset to apply to the current setpoint (positive raises the hood)
     */
    public void bumpSetpointImmediate(Angle delta) {
        Angle prevDeg = smartMotor.getMechanismPositionSetpoint().orElse(getPosition());
        Angle newDeg = prevDeg.plus(delta);
        writeSetpointImmediate(newDeg);
    }

    /**
     * Command factory: returns a short run-once Command that bumps the mechanism setpoint by {@code
     * delta} when scheduled. The returned command requires this subsystem.
     */
    public Command bumpSetpoint(Angle delta) {
        return Commands.runOnce(() -> bumpSetpointImmediate(delta), this).withName("Hood.bumpSetpoint");
    }

    // endregion

    // region Triggers & events

    /** Trigger active when the hood is within the configured position tolerance of the setpoint. */
    public final Trigger atSetpoint =
            new Trigger(
                    () -> {
                        Angle tgt = inputs.setpoint;
                        double diff = Math.abs(getPosition().in(Degrees) - tgt.in(Degrees));
                        return diff <= kPositionTolerance.in(Degrees);
                    });

    // endregion

    // (Lifecycle / periodic handled above)
}
