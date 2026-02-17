package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
import yams.motorcontrollers.local.SparkWrapper;
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;

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

    private final SparkFlex pivotMotor =
            new SparkFlex(RobotMap.Shooter.Turret.kMotorId, SparkFlex.MotorType.kBrushless);

    private final SmartMotorControllerConfig motorConfig;

    /** The SmartMotorController abstraction that allows for hardware/sim parity. */
    private final SmartMotorController smartMotor;

    /* High-level mechanism configuration */
    private final PivotConfig turretConfig;

    private final Pivot turret;

    // Track whether EasyCRT has been run to avoid re-seeding accidentally
    private boolean easyCrtInitialized = false;

    // Retry counters for periodic auto-initialization (avoid blocking in constructor)
    private int easyCrtAttempts = 0;
    private static final int EASY_CRT_MAX_ATTEMPTS = 10; // total attempts before giving up
    private int easyCrtPeriodicCounter = 0; // counts periodic loops between attempts

    // PWM duty-cycle absolute encoder wired to the RoboRIO for turret absolute position
    private final DutyCycleEncoder turretPwmEncoder =
            new DutyCycleEncoder(RobotMap.Shooter.Turret.kAbsoluteEncoderPwmChannel);

    /**
     * Updates the AdvantageKit "inputs" by refreshing hardware signals. Synchronizes TalonFX signals
     * to ensure telemetry is time-aligned.
     */
    private void updateInputs() {
        // Update turret inputs from mechanism (hardware signals are handled by the SMC wrapper)
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
                        .withSimClosedLoopController(
                                TurretConstants.kP_sim, TurretConstants.kI_sim, TurretConstants.kD_sim)
                        // Feedforward Constants
                        .withFeedforward(
                                new SimpleMotorFeedforward(
                                        TurretConstants.kS, TurretConstants.kV, TurretConstants.kA))
                        .withSimFeedforward(
                                new SimpleMotorFeedforward(
                                        TurretConstants.kS_sim, TurretConstants.kV_sim, TurretConstants.kA_sim))
                        // Telemetry
                        .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH)
                        .withGearing(TurretConstants.kGearing)
                        .withMotorInverted(TurretConstants.kMotorInverted)
                        .withIdleMode(MotorMode.BRAKE)
                        .withStatorCurrentLimit(TurretConstants.kStatorCurrentLimit);

        smartMotor = new SparkWrapper(pivotMotor, DCMotor.getNEO(1), motorConfig);

        turretConfig =
                new PivotConfig(smartMotor)
                        .withMOI(TurretConstants.kMOI)
                        .withTelemetry("TurretMech", TelemetryVerbosity.HIGH)
                        // Provide a starting position so the Pivot has a known angle at init (required by YAMS)
                        .withStartingPosition(TurretConstants.kStartingPosition)
                        .withHardLimit(TurretConstants.kHardLimitMin, TurretConstants.kHardLimitMax)
                        .withSoftLimits(TurretConstants.kSoftLimitMin, TurretConstants.kSoftLimitMax);

        turret = new Pivot(turretConfig);

        // No direct Phoenix status signals to optimize for SparkFlex here.
    }

    /**
     * Run EasyCRT once to resolve absolute mechanism angle from two absolute encoders and seed the
     * SmartMotorController / YAMS pivot with the resolved mechanism angle.
     *
     * <p>Call this once at startup (after sensors are ready). The PWM supplier should return the
     * RoboRIO-connected absolute encoder reading in rotations (0..1) as an {@link Angle}.
     */
    public void initializeEasyCRT() {
        if (easyCrtInitialized) {
            return;
        }

        // Snapshot the two absolute encoders immediately to avoid latency between reads.
        Angle abs1;
        try {
            // REV/Spark absolute encoder on the SparkFlex returns rotations (0..1).
            double raw = pivotMotor.getAbsoluteEncoder().getPosition();
            // Basic sanity check: ensure we don't propagate NaN/Infinity into the solver
            if (!Double.isFinite(raw)) {
                Logger.recordOutput("EasyCRT/Status", "SparkAbsReadInvalid");
                return;
            }
            abs1 = Degrees.of(raw * 360.0);
        } catch (RuntimeException e) {
            // Narrow catch to runtime issues (missing device, API problem). Avoid catching
            // Errors (e.g. linkage issues) which should surface during development.
            Logger.recordOutput(
                    "EasyCRT/Status",
                    "SparkAbsReadFailure:" + e.getClass().getSimpleName() + ":" + e.getMessage());
            return;
        }

        final Angle abs2;
        try {
            double rawPwm = this.turretPwmEncoder.get(); // returns duty-cycle fraction 0..1
            if (!Double.isFinite(rawPwm)) {
                Logger.recordOutput("EasyCRT/Status", "PWMAbsReadInvalid");
                return;
            }
            abs2 = Degrees.of(rawPwm * 360.0);
        } catch (RuntimeException e) {
            Logger.recordOutput(
                    "EasyCRT/Status",
                    "PWMAbsReadFailure:" + e.getClass().getSimpleName() + ":" + e.getMessage());
            return;
        }

        // Wrap snapshot values in suppliers so EasyCRT sees a consistent pair
        Supplier<Angle> s1 = () -> abs1;
        Supplier<Angle> s2 = () -> abs2;

        // Build the EasyCRT config using the requested builder-style API: supply the two
        // absolute-encoder snapshots and then configure gearing, mechanism range, and
        // inversion flags. This mirrors the user's preferred example.
        EasyCRTConfig config =
                new EasyCRTConfig(s1, s2)
                        .withAbsoluteEncoder1Gearing(
                                TurretConstants.kEasyCrtEncoder1DriverTeeth, TurretConstants.kTurretDrivenTeeth)
                        .withAbsoluteEncoder2Gearing(
                                TurretConstants.kTurretMotorDriverTeeth, TurretConstants.kTurretDrivenTeeth)
                        .withMechanismRange(
                                TurretConstants.kEasyCrtMechanismRangeMin,
                                TurretConstants.kEasyCrtMechanismRangeMax)
                        .withAbsoluteEncoderInversions(
                                TurretConstants.kEasyCrtAbs1Inverted, TurretConstants.kEasyCrtAbs2Inverted);

        // Optionally run the gear recommender in simulation to propose pinion pairs.
        if (RobotBase.isSimulation()) {
            config.withCrtGearRecommendationConstraints(
                    TurretConstants.kCrtGearRecCoverage,
                    TurretConstants.kCrtGearRecMinTeeth,
                    TurretConstants.kCrtGearRecMaxTeeth,
                    TurretConstants.kCrtGearRecMaxCompoundTeeth);
        }

        EasyCRT solver = new EasyCRT(config);
        var opt = solver.getAngleOptional();
        if (opt.isPresent()) {
            Angle mechAngle = opt.get();
            // Seed the SmartMotorController so closed-loop control starts at the correct absolute angle
            smartMotor.setEncoderPosition(mechAngle);
            Logger.recordOutput("EasyCRT/Status", "OK");
            easyCrtInitialized = true;
        } else {
            Logger.recordOutput("EasyCRT/Status", solver.getLastStatus().toString());
            Logger.recordOutput("EasyCRT/LastErrorRot", solver.getLastErrorRotations());
            Logger.recordOutput("EasyCRT/Iterations", solver.getLastIterations());
        }
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
        // Clamp requested angle to configured soft limits
        double requestedDeg = angle.in(Degrees);
        double minDeg = TurretConstants.kSoftLimitMin.in(Degrees);
        double maxDeg = TurretConstants.kSoftLimitMax.in(Degrees);
        double clampedDeg = Math.max(minDeg, Math.min(maxDeg, requestedDeg));
        Angle clamped = Degrees.of(clampedDeg);
        return turret.setAngle(clamped);
    }

    /**
     * Sets the duty cycle (percent output) for the turret.
     *
     * @param dutyCycle The output percentage (-1.0 to 1.0).
     * @return A command to run the turret at the specified duty cycle.
     */
    public Command setDutyCycle(double dutyCycle) {
        // Allow open-loop duty outputs; mechanism-level hard/soft limits are applied via PivotConfig
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
                    Angle a = angle.get();
                    return a;
                });
    }

    /** Advance the turret simulation model by one simulation tick. */
    @Override
    public void simulationPeriodic() {
        turret.simIterate();
    }

    /**
     * Public Trigger active when the turret is within the configured position tolerance of the
     * current setpoint.
     */
    public final Trigger atSetpoint =
            new Trigger(
                    () -> {
                        var tgt = turretInputs.setpoint;
                        double diff = Math.abs(getPosition().in(Degrees) - tgt.in(Degrees));
                        return diff <= TurretConstants.kPositionTolerance.in(Degrees);
                    });

    @Override
    public void periodic() {
        // Attempt a one-shot EasyCRT initialization from the turret itself. We retry a few
        // times with a small spacing between attempts in case absolute encoders are not ready
        // immediately after construction (cold-power-up behavior).
        if (!easyCrtInitialized && easyCrtAttempts < EASY_CRT_MAX_ATTEMPTS) {
            easyCrtPeriodicCounter++;
            // try roughly every 10 periodic cycles (~0.2s at 50Hz)
            if (easyCrtPeriodicCounter >= 10) {
                easyCrtPeriodicCounter = 0;
                easyCrtAttempts++;
                initializeEasyCRT();
            }
        }

        updateInputs();
        Logger.processInputs("Shooter/Turret", turretInputs);
        turret.updateTelemetry();
    }
}
