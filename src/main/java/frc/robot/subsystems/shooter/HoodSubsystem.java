package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.ShooterConstants.HoodConstants.kD;
import static frc.robot.constants.ShooterConstants.HoodConstants.kD_sim;
import static frc.robot.constants.ShooterConstants.HoodConstants.kGearing;
import static frc.robot.constants.ShooterConstants.HoodConstants.kHardLimitMax;
import static frc.robot.constants.ShooterConstants.HoodConstants.kHardLimitMin;
import static frc.robot.constants.ShooterConstants.HoodConstants.kI;
import static frc.robot.constants.ShooterConstants.HoodConstants.kI_sim;
import static frc.robot.constants.ShooterConstants.HoodConstants.kMotorInverted;
import static frc.robot.constants.ShooterConstants.HoodConstants.kP;
import static frc.robot.constants.ShooterConstants.HoodConstants.kP_sim;
import static frc.robot.constants.ShooterConstants.HoodConstants.kSoftLimitMax;
import static frc.robot.constants.ShooterConstants.HoodConstants.kSoftLimitMin;
import static frc.robot.constants.ShooterConstants.HoodConstants.kStartingPosition;
import static frc.robot.constants.ShooterConstants.HoodConstants.kStatorCurrentLimit;
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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.RobotMap;
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
                        .withControlMode(ControlMode.CLOSED_LOOP)
                        .withClosedLoopController(kP, kI, kD)
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
                        // Ensure Arm has a known starting angle for simulation and replay
                        .withStartingPosition(kStartingPosition)
                        .withHardLimit(kHardLimitMin, kHardLimitMax)
                        .withSoftLimits(kSoftLimitMin, kSoftLimitMax);

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

    public Command setAngle(Angle angle) {
        return hood.setAngle(angle);
    }

    /** Supplier-backed overload for dynamic angles. */
    public Command setAngle(Supplier<Angle> angle) {
        return hood.setAngle(angle);
    }

    public Command setDutyCycle(double dutyCycle) {
        // Allow open-loop duty outputs; mechanism-level hard/soft limits are applied
        // via ArmConfig
        return hood.set(dutyCycle);
    }

    private void writeSetpointImmediate(Angle target) {
        hood.setMechanismPositionSetpoint(target);
    }

    public void bumpSetpointImmediate(Angle delta) {
        Angle prevDeg = smartMotor.getMechanismPositionSetpoint().orElse(getPosition());
        Angle newDeg = prevDeg.plus(delta);
        writeSetpointImmediate(newDeg);
    }

    public Command bumpSetpoint(Angle delta) {
        return Commands.runOnce(() -> bumpSetpointImmediate(delta), this).withName("Hood.bumpSetpoint");
    }

    // endregion

    // region Triggers & events

    // (Lifecycle / periodic handled above)
}
