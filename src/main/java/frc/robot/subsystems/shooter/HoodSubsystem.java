package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.ShooterConstants.HoodConstants.kD;
import static frc.robot.constants.ShooterConstants.HoodConstants.kD_sim;
import static frc.robot.constants.ShooterConstants.HoodConstants.kGearing;
import static frc.robot.constants.ShooterConstants.HoodConstants.kHardLimitMax;
import static frc.robot.constants.ShooterConstants.HoodConstants.kHardLimitMin;
import static frc.robot.constants.ShooterConstants.HoodConstants.kI;
import static frc.robot.constants.ShooterConstants.HoodConstants.kI_sim;
import static frc.robot.constants.ShooterConstants.HoodConstants.kLength;
import static frc.robot.constants.ShooterConstants.HoodConstants.kMass;
import static frc.robot.constants.ShooterConstants.HoodConstants.kMotorInverted;
import static frc.robot.constants.ShooterConstants.HoodConstants.kP;
import static frc.robot.constants.ShooterConstants.HoodConstants.kP_sim;
import static frc.robot.constants.ShooterConstants.HoodConstants.kSoftLimitMax;
import static frc.robot.constants.ShooterConstants.HoodConstants.kSoftLimitMin;
import static frc.robot.constants.ShooterConstants.HoodConstants.kStartingPosition;
import static frc.robot.constants.ShooterConstants.HoodConstants.kStatorCurrentLimit;
import static frc.robot.constants.ShooterConstants.HoodConstants.kTolerance;
import static frc.robot.constants.ShooterConstants.HoodConstants.pivotFeedforward;
import static frc.robot.constants.ShooterConstants.HoodConstants.pivotFeedforwardSim;
import static frc.robot.constants.TelemetryKeys.kHoodMechTelemetry;
import static frc.robot.constants.TelemetryKeys.kHoodMotorTelemetry;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.RobotMap;
import frc.robot.constants.ShooterConstants.HoodConstants;
import frc.robot.util.PhoenixUtil;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.remote.TalonFXWrapper;

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

    // Hood subsystem: positional pivot controlled via YAMS Pivot. This subsystem exposes
    // command-returning APIs (setAngle, runTo) and keeps telemetry updated. There are no
    // hardware limit-switches assumed here; any external limits are enforced via the
    // mechanism soft/hard limits configured in ShooterConstants.

    // endregion

    // region Controller configuration / mechanism

    private final SmartMotorControllerConfig motorConfig;
    private final SmartMotorController smartMotor;
    private final PivotConfig hoodConfig;
    private final Pivot hood;

    // endregion

    public HoodSubsystem() {
        motorConfig =
                new SmartMotorControllerConfig(this)
                        .withMechanismCircumference(Inches.of(1.26875 * Math.PI))
                        .withControlMode(ControlMode.CLOSED_LOOP)
                        .withClosedLoopController(
                                kP, kI, kD, DegreesPerSecond.of(270), DegreesPerSecondPerSecond.of(270))
                        .withSimClosedLoopController(kP_sim, kI_sim, kD_sim)
                        .withFeedforward(pivotFeedforward())
                        .withSimFeedforward(pivotFeedforwardSim())
                        .withTelemetry(kHoodMotorTelemetry, Constants.telemetryVerbosity())
                        .withGearing(kGearing)
                        .withMotorInverted(kMotorInverted)
                        .withIdleMode(MotorMode.BRAKE)
                        .withStatorCurrentLimit(kStatorCurrentLimit);

        smartMotor = new TalonFXWrapper(motor, DCMotor.getKrakenX44Foc(1), motorConfig);

        hoodConfig =
                new PivotConfig(smartMotor)
                        .withTelemetry(kHoodMechTelemetry, Constants.telemetryVerbosity())
                        .withMOI(kLength, kMass)
                        // Ensure Arm has a known starting angle for simulation and replay
                        .withStartingPosition(kStartingPosition)
                        .withHardLimit(kHardLimitMin, kHardLimitMax)
                        .withSoftLimits(kSoftLimitMin, kSoftLimitMax)
                        .withMOI(kLength, kMass);

        hood = new Pivot(hoodConfig);

        // Optimize CAN update frequency for the signals we use (centralized default)
        BaseStatusSignal.setUpdateFrequencyForAll(
                Constants.CommsConstants.DEFAULT_TELEMETRY_HZ, positionSignal, referenceSignal);
        motor.getVelocity().setUpdateFrequency(0);
    }

    // region Lifecycle / periodic

    private void updateInputs() {
        // Refresh Phoenix signals so logged telemetry is time-aligned with hardware.
        PhoenixUtil.refresh(positionSignal, referenceSignal);

        // Record raw Phoenix signals.
        Logger.recordOutput("Hood/FX/PositionDegrees", positionSignal.getValue().in(Degrees));
        Logger.recordOutput(
                "Hood/FX/ReferenceDegrees", Degrees.convertFrom(referenceSignal.getValue(), Rotations));

        inputs.angle = hood.getAngle();
        inputs.volts = smartMotor.getVoltage();
        inputs.current = smartMotor.getStatorCurrent();
        inputs.setpoint = smartMotor.getMechanismPositionSetpoint().orElse(Degrees.of(0));
        Logger.recordOutput(
                "Mechanisms/HoodIsMoving",
                Math.abs(inputs.setpoint.in(Degrees) - inputs.angle.in(Degrees)) > 1.0);
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

    /**
     * Returns a command that moves the hood to the requested absolute angle and holds that angle
     * while the command is scheduled.
     *
     * @param angle target hood angle (mechanism units)
     * @return a command that sets and holds the hood angle while scheduled
     */
    public Command setAngle(Angle angle) {
        return hood.setAngle(angle);
    }

    /**
     * Supplier-backed overload for dynamic angles.
     *
     * <p>The supplied angle is read while the returned command is active. Use this when the target
     * depends on runtime values (tunables, operators, or computed solutions).
     *
     * @param angle supplier of target hood angle
     * @return a command that continuously applies the supplied target while scheduled
     */
    public Command setAngle(Supplier<Angle> angle) {
        return hood.setAngle(angle);
    }

    /**
     * Returns an open-loop duty-cycle command for the hood pivot.
     *
     * <p>This bypasses closed-loop positional control and applies a percent output to the mechanism.
     * Mechanism-level soft/hard limits defined in {@link HoodConstants} are still enforced by the
     * underlying Pivot configuration.
     *
     * @param dutyCycle open-loop output (-1.0 .. 1.0)
     * @return a command that applies the given duty cycle while scheduled
     */
    public Command setDutyCycle(double dutyCycle) {
        // Allow open-loop duty outputs; mechanism-level hard/soft limits are applied
        // via PivotConfig (the underlying Pivot configuration enforces limits)
        return hood.set(dutyCycle);
    }

    /**
     * Move the hood to the requested angle and finish when the mechanism reaches the target within
     * the configured tolerance.
     *
     * <p>This is a blocking command that completes once the hood's position is within {@link
     * HoodConstants#kTolerance} of the target.
     *
     * @param angle target hood angle
     * @return a command that completes when the hood reaches the target
     */
    public Command runTo(Angle angle) {
        return hood.runTo(angle, kTolerance);
    }

    // endregion
}
