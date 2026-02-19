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
 * Hood pivot subsystem for controlling the shooting angle.
 *
 * <p>Modeled as a positional arm using the YAMS Arm abstraction and TalonFX motor controller.
 */
public class HoodSubsystem extends SubsystemBase {

    @AutoLog
    public static class HoodInputs {
        public Angle angle = Degrees.of(0);
        public Angle setpoint = Degrees.of(0);
        public Voltage volts = Volts.of(0);
        public Current current = Amps.of(0);
    }

    private final HoodInputsAutoLogged inputs = new HoodInputsAutoLogged();

    /* Hardware */
    private final TalonFX motor = new TalonFX(RobotMap.Shooter.Hood.kMotorId);

    /* Signals */
    private final StatusSignal<Angle> positionSignal = motor.getPosition();
    private final StatusSignal<Double> referenceSignal = motor.getClosedLoopReference();

    /* Controller config */
    private final SmartMotorControllerConfig motorConfig;
    private final SmartMotorController smartMotor;
    private final ArmConfig hoodConfig;
    private final Arm hood;
    /** Current commanded target for the hood; used by the default hold command. */
    private volatile Angle currentTarget;

    // Explicit Phoenix refreshes are required for telemetry; call directly.

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

        // Initialize the currentTarget to a clamped starting angle (avoid commanding
        // outside the configured soft limits at startup which can cause unwanted motion).
        double startMeasuredDeg = hood.getAngle().in(Degrees);
        double clampedStartDeg =
                Math.max(kSoftLimitMin.in(Degrees), Math.min(kSoftLimitMax.in(Degrees), startMeasuredDeg));
        currentTarget = Degrees.of(clampedStartDeg);
        // Optimize CAN update frequency for the signals we use (centralized default)
        BaseStatusSignal.setUpdateFrequencyForAll(
                Constants.CommsConstants.DEFAULT_TELEMETRY_HZ, positionSignal, referenceSignal);
        motor.getVelocity().setUpdateFrequency(0);
    }

    private void updateInputs() {
        // Refresh Phoenix signals to ensure telemetry is up-to-date for AdvantageKit/YAMS
        BaseStatusSignal.refreshAll(positionSignal, referenceSignal);

        inputs.angle = hood.getAngle();
        inputs.volts = smartMotor.getVoltage();
        inputs.current = smartMotor.getStatorCurrent();
        inputs.setpoint = smartMotor.getMechanismPositionSetpoint().orElse(Degrees.of(0));
    }

    public Angle getPosition() {
        return inputs.angle;
    }

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

    public Command setDutyCycle(double dutyCycle) {
        // Allow open-loop duty outputs; mechanism-level hard/soft limits are applied via ArmConfig
        return hood.set(dutyCycle);
    }

    public Command setAngle(Supplier<Angle> angle) {
        return hood.setAngle(
                () -> {
                    Angle a = angle.get();
                    return a;
                });
    }

    /**
     * Convenience helper to bump the current hood setpoint by the provided delta (signed). This
     * schedules a short-lived command that updates the positional controller's target to (current
     * setpoint + delta).
     */
    public void bumpBy(Angle delta) {
        // Compute the new target relative to the current commanded target. This avoids
        // accumulating error if the mechanism is still moving; bumping the commanded
        // setpoint is the simplest, most deterministic behavior for button presses.
        double newDeg = currentTarget.in(Degrees) + delta.in(Degrees);
        double minDeg = kSoftLimitMin.in(Degrees);
        double maxDeg = kSoftLimitMax.in(Degrees);
        double clampedDeg = Math.max(minDeg, Math.min(maxDeg, newDeg));
        currentTarget = Degrees.of(clampedDeg);
        // Immediately schedule a positional command to apply the new target so the
        // adjustment takes effect even if other commands or default behaviors exist.
        // Use a fixed/clamped Angle here (not a Supplier) so the scheduled command
        // holds an immutable setpoint and is not affected by subsequent changes to
        // `currentTarget` while the command is running.
        Angle fixedTarget = Degrees.of(clampedDeg);
        // Schedule the positional command directly (no console logging).
        final Command inner = hood.setAngle(fixedTarget);
        // Use the Command API to schedule rather than accessing the scheduler singleton.
        inner.schedule();
    }

    /** Returns the current commanded target for the hood. */
    public Angle getTarget() {
        return currentTarget;
    }

    @Override
    public void simulationPeriodic() {
        hood.simIterate();
    }

    /**
     * Public Trigger active when the hood is within the configured position tolerance of the current
     * setpoint.
     */
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
        hood.updateTelemetry();
    }
}
