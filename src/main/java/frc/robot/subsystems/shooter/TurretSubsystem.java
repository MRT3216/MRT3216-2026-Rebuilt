package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.shooter.ShooterConstants.TurretConstants.kA;
import static frc.robot.subsystems.shooter.ShooterConstants.TurretConstants.kA_sim;
import static frc.robot.subsystems.shooter.ShooterConstants.TurretConstants.kCRTCommonRatio;
import static frc.robot.subsystems.shooter.ShooterConstants.TurretConstants.kCRTDriveGearTeeth;
import static frc.robot.subsystems.shooter.ShooterConstants.TurretConstants.kCRTEncoder1Inverted;
import static frc.robot.subsystems.shooter.ShooterConstants.TurretConstants.kCRTEncoder1Offset;
import static frc.robot.subsystems.shooter.ShooterConstants.TurretConstants.kCRTEncoder1PinionTeeth;
import static frc.robot.subsystems.shooter.ShooterConstants.TurretConstants.kCRTEncoder2Inverted;
import static frc.robot.subsystems.shooter.ShooterConstants.TurretConstants.kCRTEncoder2Offset;
import static frc.robot.subsystems.shooter.ShooterConstants.TurretConstants.kCRTEncoder2PinionTeeth;
import static frc.robot.subsystems.shooter.ShooterConstants.TurretConstants.kCRTMatchTolerance;
import static frc.robot.subsystems.shooter.ShooterConstants.TurretConstants.kCRTMechanismMax;
import static frc.robot.subsystems.shooter.ShooterConstants.TurretConstants.kCRTMechanismMin;
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
import static frc.robot.subsystems.shooter.ShooterConstants.TurretConstants.kUseCRT;
import static frc.robot.subsystems.shooter.ShooterConstants.TurretConstants.kV;
import static frc.robot.subsystems.shooter.ShooterConstants.TurretConstants.kV_sim;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.constants.RobotMap;
import java.util.Optional;
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
import yams.units.EasyCRT;
import yams.units.EasyCRT.CRTStatus;
import yams.units.EasyCRTConfig;

/**
 * AdvantageKit-ready Turret Subsystem for MRT 3216.
 *
 * <p>This subsystem manages a single-NEO turret pivot using the YAMS library and RevLib. It
 * utilizes an IO-layer abstraction for full log replay capabilities, ensuring that hardware states
 * (Inputs) are separated from software commands (Outputs).
 *
 * <h3>Wrap-around behavior</h3>
 *
 * The turret's travel range is defined by {@code kSoftLimitMin} / {@code kSoftLimitMax} (currently
 * ±190°, 380° total). When the shot solver or stick input requests an angle, {@link
 * #wrapAngle(Angle)} selects the ±360° equivalent <em>closest to the turret's current position</em>
 * and clamps to the soft limits. This prevents unnecessary full-rotation swings when {@code atan2}
 * wraps at ±180° — a common pitfall since our travel exceeds 360° coverage by 20°.
 *
 * <p>The algorithm naturally handles both symmetric limits (±190°) and asymmetric limits (e.g.,
 * −60° / +320° if the turret's encoder zero isn't straight-forward). Only the soft-limit constants
 * need to change.
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

    /** CRT status string logged once at boot, then every loop in telemetry. */
    private CRTStatus crtStatus = CRTStatus.NOT_ATTEMPTED;

    private double crtResolvedDeg = Double.NaN;
    private double crtErrorRot = Double.NaN;
    private int crtIterations = 0;

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

        // ── EasyCRT absolute-position bootstrapping ──
        // When enabled, uses two absolute encoders with different gear ratios on the 90T ring
        // gear to resolve the turret's true position via the Chinese Remainder Theorem at boot.
        // Encoder 1: REV Through Bore on SparkMax abs-encoder port (13T pinion).
        // Encoder 2: PWM absolute encoder on RoboRIO DIO (10T pinion).
        // Falls back to kStartingPosition if CRT is disabled or the solve fails.
        Angle startingPosition = kStartingPosition;

        if (kUseCRT) {
            Optional<Angle> crtResult = attemptCRTSolve();
            if (crtResult.isPresent()) {
                startingPosition = crtResult.get();
            } else {
                DriverStation.reportWarning(
                        "[Turret CRT] Solve failed (status="
                                + crtStatus
                                + "). Falling back to kStartingPosition="
                                + kStartingPosition,
                        false);
            }
        } else {
            crtStatus = CRTStatus.NOT_ATTEMPTED;
            crtResolvedDeg = Double.NaN;
            crtErrorRot = Double.NaN;
            crtIterations = 0;
        }

        PivotConfig turretConfig =
                new PivotConfig(smartMotor)
                        .withStartingPosition(startingPosition)
                        .withTelemetry(kTurretMechTelemetry, Constants.telemetryVerbosity())
                        .withMOI(kMOI)
                        .withHardLimit(kHardLimitMin, kHardLimitMax)
                        .withSoftLimits(kSoftLimitMin, kSoftLimitMax);

        turret = new Pivot(turretConfig);

        // Log CRT results at boot for easy inspection.
        Logger.recordOutput("Shooter/Turret/CRT/Status", crtStatus.name());
        Logger.recordOutput("Shooter/Turret/CRT/ResolvedDeg", crtResolvedDeg);
        Logger.recordOutput("Shooter/Turret/CRT/ErrorRot", crtErrorRot);
        Logger.recordOutput("Shooter/Turret/CRT/Iterations", crtIterations);
        Logger.recordOutput("Shooter/Turret/CRT/Enabled", kUseCRT);
    }

    /**
     * Attempts to resolve the turret's absolute position using EasyCRT.
     *
     * <p>Reads two absolute encoders that mesh with the 90T turret ring gear through different pinion
     * sizes:
     *
     * <ul>
     *   <li><b>Encoder 1 (13T pinion):</b> REV Through Bore plugged into the SparkMax's
     *       absolute-encoder port. Read via {@code pivotMotor.getAbsoluteEncoder().getPosition()}
     *       (returns 0–1 rotations).
     *   <li><b>Encoder 2 (10T pinion):</b> Absolute encoder wired to a RoboRIO DIO channel as a PWM
     *       duty-cycle signal. Read via {@code turretPwmEncoder.get()} (returns 0–1 rotations).
     * </ul>
     *
     * <p>Configures the CRT solver with the turret's ring-gear / pinion gearing and runs a single
     * solve. Stores diagnostic fields ({@link #crtStatus}, {@link #crtErrorRot}, {@link
     * #crtIterations}) for telemetry regardless of outcome.
     *
     * @return the resolved mechanism angle, or empty if the solve fails.
     */
    private Optional<Angle> attemptCRTSolve() {
        // Encoder 1: SparkMax absolute encoder (13T pinion on 90T ring gear)
        Supplier<Angle> enc1Supplier =
                () -> Rotations.of(pivotMotor.getAbsoluteEncoder().getPosition());

        // Encoder 2: RoboRIO PWM DutyCycleEncoder (10T pinion on 90T ring gear).
        // Created locally so DIO channel 0 is not claimed when CRT is disabled.
        DutyCycleEncoder pwmEncoder =
                new DutyCycleEncoder(RobotMap.Shooter.Turret.kAbsoluteEncoderPwmChannel);
        Supplier<Angle> enc2Supplier = () -> Rotations.of(pwmEncoder.get());

        // Build EasyCRT config
        EasyCRTConfig config =
                new EasyCRTConfig(enc1Supplier, enc2Supplier)
                        .withCommonDriveGear(
                                kCRTCommonRatio,
                                kCRTDriveGearTeeth,
                                kCRTEncoder1PinionTeeth,
                                kCRTEncoder2PinionTeeth)
                        .withMechanismRange(kCRTMechanismMin, kCRTMechanismMax)
                        .withAbsoluteEncoderOffsets(kCRTEncoder1Offset, kCRTEncoder2Offset)
                        .withAbsoluteEncoderInversions(kCRTEncoder1Inverted, kCRTEncoder2Inverted)
                        .withMatchTolerance(kCRTMatchTolerance);

        // Solve once
        EasyCRT solver = new EasyCRT(config);
        Optional<Angle> result = solver.getAngleOptional();

        // Capture diagnostics
        crtStatus = solver.getLastStatus();
        crtErrorRot = solver.getLastErrorRotations();
        crtIterations = solver.getLastIterations();
        crtResolvedDeg = result.map(a -> a.in(Degrees)).orElse(Double.NaN);

        return result;
    }

    // endregion

    // region Lifecycle

    /**
     * Updates the AdvantageKit "inputs" by refreshing hardware signals. Synchronizes signals to
     * ensure telemetry is time-aligned.
     */
    private void updateInputs() {
        turretInputs.angle = turret.getAngle();
        Logger.recordOutput("Shooter/Turret/PositionDegrees", turretInputs.angle.in(Degrees));
        turretInputs.volts = smartMotor.getVoltage();
        turretInputs.current = smartMotor.getStatorCurrent();
        turretInputs.setpoint = smartMotor.getMechanismPositionSetpoint().orElse(Degrees.of(0));
        Logger.recordOutput(
                "Mechanisms/TurretIsMoving",
                Math.abs(turretInputs.setpoint.in(Degrees) - turretInputs.angle.in(Degrees)) > 1.0);

        Robot.batteryLogger.reportCurrentUsage("Turret", turretInputs.current.in(Amps));
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
     * Wraps a requested turret angle into the soft-limit range [{@code kSoftLimitMin}, {@code
     * kSoftLimitMax}], choosing the representation closest to the turret's <em>current</em> encoder
     * position to avoid unnecessary full-rotation swings.
     *
     * <h4>Why position-aware?</h4>
     *
     * The shot solver ({@link frc.robot.util.shooter.HybridTurretUtil}) computes a robot-relative
     * azimuth via {@code atan2}, which returns values in (−180°, 180°]. Our turret can reach ±190°,
     * so there is a 10° band on each side (180°–190°) that {@code atan2} maps to the opposite sign.
     * For example, a target at +181° from robot-forward is reported as −179° by {@code atan2}. If the
     * turret is currently at +185°, a naïve wrap would command −179° — a 364° swing instead of a 4°
     * move. By considering the ±360° equivalents and picking the one nearest the current position,
     * the turret moves the short way and only wraps when it truly needs to cross the dead zone.
     *
     * <h4>Algorithm</h4>
     *
     * <ol>
     *   <li>Normalize the requested angle into (−360°, 360°) via modulo.
     *   <li>Generate three candidates: {@code normalized}, {@code normalized + 360°}, {@code
     *       normalized − 360°}.
     *   <li>Select the candidate closest to the current turret position.
     *   <li>Clamp to [{@code kSoftLimitMin}, {@code kSoftLimitMax}].
     * </ol>
     *
     * <p>The final clamp handles the rear dead zone gracefully — if all candidates fall outside the
     * soft limits, the turret drives as close as it can.
     *
     * @param requested the raw target angle (any range, typically from the shot solver or stick
     *     input).
     * @return the wrapped Angle, guaranteed within [{@code kSoftLimitMin}, {@code kSoftLimitMax}].
     */
    public Angle wrapAngle(Angle requested) {
        double minDeg = kSoftLimitMin.in(Degrees);
        double maxDeg = kSoftLimitMax.in(Degrees);
        double currentDeg = getPosition().in(Degrees);

        // Normalize into (-360, 360)
        double deg = requested.in(Degrees) % 360.0;

        // Generate three candidates and pick the one nearest the current position
        double[] candidates = {deg, deg + 360.0, deg - 360.0};
        double best = candidates[0];
        double bestDist = Math.abs(candidates[0] - currentDeg);
        for (int i = 1; i < candidates.length; i++) {
            double dist = Math.abs(candidates[i] - currentDeg);
            if (dist < bestDist) {
                best = candidates[i];
                bestDist = dist;
            }
        }

        // Clamp to soft limits
        return Degrees.of(Math.max(minDeg, Math.min(maxDeg, best)));
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
