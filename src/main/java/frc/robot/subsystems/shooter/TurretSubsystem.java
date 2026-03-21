package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.shooter.ShooterConstants.TurretConstants.kA;
import static frc.robot.subsystems.shooter.ShooterConstants.TurretConstants.kA_sim;
import static frc.robot.subsystems.shooter.ShooterConstants.TurretConstants.kD;
import static frc.robot.subsystems.shooter.ShooterConstants.TurretConstants.kD_sim;
import static frc.robot.subsystems.shooter.ShooterConstants.TurretConstants.kGearing;
import static frc.robot.subsystems.shooter.ShooterConstants.TurretConstants.kHardLimitMax;
import static frc.robot.subsystems.shooter.ShooterConstants.TurretConstants.kHardLimitMin;
import static frc.robot.subsystems.shooter.ShooterConstants.TurretConstants.kI;
import static frc.robot.subsystems.shooter.ShooterConstants.TurretConstants.kI_sim;
import static frc.robot.subsystems.shooter.ShooterConstants.TurretConstants.kMOI;
import static frc.robot.subsystems.shooter.ShooterConstants.TurretConstants.kMaxAccel;
import static frc.robot.subsystems.shooter.ShooterConstants.TurretConstants.kMaxVelocity;
import static frc.robot.subsystems.shooter.ShooterConstants.TurretConstants.kMotorInverted;
import static frc.robot.subsystems.shooter.ShooterConstants.TurretConstants.kP;
import static frc.robot.subsystems.shooter.ShooterConstants.TurretConstants.kP_sim;
import static frc.robot.subsystems.shooter.ShooterConstants.TurretConstants.kS;
import static frc.robot.subsystems.shooter.ShooterConstants.TurretConstants.kS_sim;
import static frc.robot.subsystems.shooter.ShooterConstants.TurretConstants.kSoftLimitMax;
import static frc.robot.subsystems.shooter.ShooterConstants.TurretConstants.kSoftLimitMin;
import static frc.robot.subsystems.shooter.ShooterConstants.TurretConstants.kStartingPosition;
import static frc.robot.subsystems.shooter.ShooterConstants.TurretConstants.kStatorCurrentLimit;
import static frc.robot.subsystems.shooter.ShooterConstants.TurretConstants.kV;
import static frc.robot.subsystems.shooter.ShooterConstants.TurretConstants.kV_sim;
import static frc.robot.subsystems.shooter.ShooterConstants.TurretConstants.kWrapThreshold;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
 *
 * <h3>Wrap-around behavior</h3>
 *
 * The turret can physically rotate ±190° from its zero-forward position. When a tracking target
 * would require passing through the mechanical limit (e.g., a target at +195° while currently at
 * +185°), the turret instead "jumps" to the opposite side of the travel range (e.g., −165°) and
 * approaches the target from the other direction. This lets the turret continuously track targets
 * that cross the rear dead-zone without stalling against a hard stop.
 *
 * <p><b>Note:</b> YAMS {@code PivotConfig.withWrapping()} cannot be used here because it calls
 * {@code SmartMotorControllerConfig.withContinuousWrapping()}, which is incompatible with soft
 * limits. Continuous wrapping is designed for infinite-rotation mechanisms (swerve azimuth), not
 * limited-travel turrets. The wrap logic is implemented manually in {@link #wrapAngle(Angle)}.
 */
public class TurretSubsystem extends SubsystemBase {
    private static final String kTurretMotorTelemetry = "TurretMotor";
    private static final String kTurretMechTelemetry = "TurretMech";

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

    // region Hardware

    private final SparkMax pivotMotor =
            new SparkMax(RobotMap.Shooter.Turret.kMotorId, SparkMax.MotorType.kBrushless);

    // endregion

    // region Controller & mechanism

    private final SmartMotorControllerConfig motorConfig;

    /** The SmartMotorController abstraction that allows for hardware/sim parity. */
    private final SmartMotorController smartMotor;

    private final Pivot turret;

    // endregion

    // region Constructor

    /** Initializes the subsystem, sets signal update frequencies, and optimizes CAN utilization. */
    public TurretSubsystem() {
        // Initialize motor controller config in constructor to avoid object-escape
        motorConfig =
                new SmartMotorControllerConfig(this)
                        .withControlMode(ControlMode.CLOSED_LOOP)
                        .withClosedLoopController(kP, kI, kD, kMaxVelocity, kMaxAccel)
                        .withSimClosedLoopController(kP_sim, kI_sim, kD_sim, kMaxVelocity, kMaxAccel)
                        .withFeedforward(new SimpleMotorFeedforward(kS, kV, kA))
                        .withSimFeedforward(new SimpleMotorFeedforward(kS_sim, kV_sim, kA_sim))
                        .withTelemetry(kTurretMotorTelemetry, Constants.telemetryVerbosity())
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

    // endregion

    // region Lifecycle

    /**
     * Updates the AdvantageKit "inputs" by refreshing hardware signals. Synchronizes signals to
     * ensure telemetry is time-aligned.
     */
    private void updateInputs() {
        turretInputs.angle = turret.getAngle();
        turretInputs.volts = smartMotor.getVoltage();
        turretInputs.current = smartMotor.getStatorCurrent();
        turretInputs.setpoint = smartMotor.getMechanismPositionSetpoint().orElse(Degrees.of(0));
    }

    @Override
    public void periodic() {
        updateInputs();
        Logger.processInputs("Shooter/Turret", turretInputs);
        turret.updateTelemetry();
    }

    /** Advance the turret simulation model by one simulation tick. */
    @Override
    public void simulationPeriodic() {
        turret.simIterate();
    }

    // endregion

    // region Wrap-around logic

    /**
     * Wraps a requested turret angle so it stays within the ±{@value
     * ShooterConstants.TurretConstants#kWrapThresholdDeg}° soft-limit range.
     *
     * <p>If the requested angle is within [-kWrapThreshold, +kWrapThreshold] it passes through
     * unchanged. If it exceeds either limit (e.g., a vision solver returning +200° for a target
     * behind the robot), the angle is shifted by ±360° to land on the opposite side of the travel
     * range.
     *
     * <p>After wrapping, if the result still falls outside the soft limits the angle is clamped. The
     * turret's 380° total travel (±190°) means there is a 20° rear dead-zone (from ±190° to ±180° on
     * the opposite side) where targets are unreachable — clamping handles that gracefully by driving
     * as close as possible.
     *
     * <p>WPILib convention: <b>CCW-positive</b>. Zero is turret-forward.
     *
     * @param requested the raw target angle (any range).
     * @return the wrapped Angle, guaranteed within [-kWrapThreshold, +kWrapThreshold].
     */
    public static Angle wrapAngle(Angle requested) {
        double threshDeg = kWrapThreshold.in(Degrees);

        // Normalize into (-360, 360) first to handle multi-revolution inputs
        double deg = requested.in(Degrees) % 360.0;

        // Bring into [-180, 180) — standard principal value
        if (deg > 180.0) {
            deg -= 360.0;
        } else if (deg <= -180.0) {
            deg += 360.0;
        }

        // At this point deg is in (-180, 180]. If it exceeds the soft limits,
        // jump 360° to the opposite side.
        if (deg > threshDeg) {
            deg -= 360.0;
        } else if (deg < -threshDeg) {
            deg += 360.0;
        }

        // Final clamp to the soft limit range — handles the rear dead-zone
        return Degrees.of(Math.max(-threshDeg, Math.min(threshDeg, deg)));
    }

    // endregion

    // region Public API

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
     * Sets the target angle using a dynamic supplier (e.g., from a Vision subsystem). The supplied
     * angle is passed through {@link #wrapAngle(Angle)} every loop so the turret will automatically
     * jump to the opposite side of its travel when the target crosses the rear dead-zone.
     *
     * @param angle A supplier providing the target Angle.
     * @return A command to track the supplier's angle with wrap-around.
     */
    public Command setAngle(Supplier<Angle> angle) {
        return Commands.run(
                        () -> {
                            smartMotor.setPosition(wrapAngle(angle.get()));
                        },
                        this)
                .withName(getName() + " SetAngle Wrapped Supplier");
    }

    /**
     * Sets the target angle for the turret. The angle is passed through {@link #wrapAngle(Angle)} so
     * out-of-range requests are mapped to the reachable side of the turret's travel.
     *
     * @param angle The target Angle.
     * @return A command to set and maintain the requested angle.
     */
    public Command setAngle(Angle angle) {
        return turret.setAngle(wrapAngle(angle));
    }

    // endregion
}
