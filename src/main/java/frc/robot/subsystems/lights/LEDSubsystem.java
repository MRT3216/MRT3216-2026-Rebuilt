package frc.robot.subsystems.lights;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.LEDsConstants;
import frc.robot.constants.RobotMap.LEDs;
import frc.robot.util.HubShiftUtil;
import java.util.function.BooleanSupplier;

/**
 * LED subsystem — drives addressable LEDs based on robot state and hub shift timing.
 *
 * <p>Pattern priority (highest first):
 *
 * <ol>
 *   <li>Disabled — slow teal/orange team wave
 *   <li>Autonomous — fast orange/cyan wave
 *   <li>Intaking — purple strobe
 *   <li>Aim lock — solid green
 *   <li>Shift ending (≤5 s remaining) — fast orange strobe warning
 *   <li>Shift active — green/black wave ("go score!")
 *   <li>Shift inactive — dim alliance color
 * </ol>
 */
public class LEDSubsystem extends SubsystemBase {
    // region Hardware

    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;

    // endregion

    // region State

    private Color allianceColor = Color.kCyan;

    /** Flags set by external commands to override the default shift-based pattern. */
    private boolean intaking = false;

    private boolean aimLock = false;

    // endregion

    // region Constants

    private static final double WAVE_EXPONENT = 0.4;
    private static final double WAVE_FAST_CYCLE_LENGTH = 25.0;
    private static final double WAVE_FAST_DURATION = 0.25;
    private static final double WAVE_ALLIANCE_CYCLE_LENGTH = 15.0;
    private static final double WAVE_ALLIANCE_DURATION = 2.0;
    private static final double STROBE_FAST_DURATION = 0.1;

    /** Seconds before a shift transition at which the warning strobe begins. */
    private static final double SHIFT_WARNING_SECS = 5.0;

    // endregion

    // region Singleton

    private static LEDSubsystem instance;

    public static LEDSubsystem getInstance() {
        if (instance == null) {
            instance = new LEDSubsystem();
        }
        return instance;
    }

    // endregion

    // region Constructor

    private LEDSubsystem() {
        led = new AddressableLED(LEDs.kPort);
        ledBuffer = new AddressableLEDBuffer(LEDsConstants.kNumLEDs);
        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();
    }

    // endregion

    // region Lifecycle

    @Override
    public void periodic() {
        // Default to off — overwritten by whichever pattern wins below.
        setColor(Color.kBlack);

        // Keep alliance color up to date when connected to FMS.
        if (DriverStation.isFMSAttached()) {
            allianceColor =
                    DriverStation.getAlliance()
                            .map(a -> a == Alliance.Blue ? Color.kCyan : Color.kRed)
                            .orElse(Color.kCyan);
        }

        if (DriverStation.isDisabled()) {
            // Slow team-spirit wave while waiting on the field.
            wave(Color.kCyan, Color.kDarkOrange, WAVE_ALLIANCE_CYCLE_LENGTH, WAVE_ALLIANCE_DURATION);
        } else if (DriverStation.isAutonomous()) {
            // Fast wave so the audience knows auto is running.
            wave(Color.kDarkOrange, Color.kCyan, WAVE_FAST_CYCLE_LENGTH, WAVE_FAST_DURATION);
        } else {
            // ── Teleop ──────────────────────────────────────────────────────
            // Command-driven overrides (intaking > aimLock) take priority,
            // then shift-aware patterns fill the gaps.
            if (intaking) {
                strobe(Color.kPurple, STROBE_FAST_DURATION);
            } else if (aimLock) {
                setColor(Color.kGreen);
            } else {
                applyShiftPattern();
            }
        }

        // Push the buffer to the LED strip.
        led.setData(ledBuffer);
    }

    // endregion

    // region Public API

    /** Command to set the intaking LED flag. */
    public Command setIntakingLEDCommand(BooleanSupplier on) {
        return this.runOnce(
                () -> {
                    clearState();
                    intaking = on.getAsBoolean();
                });
    }

    /** Command to set the aim-lock LED flag. */
    public Command setAimLockLEDCommand(BooleanSupplier on) {
        return this.runOnce(
                () -> {
                    clearState();
                    aimLock = on.getAsBoolean();
                });
    }

    // endregion

    // region Private helpers

    /**
     * Hub-shift-aware teleop pattern.
     *
     * <ul>
     *   <li>Shift ending (≤5 s) — fast orange strobe so the driver prepares for the transition
     *   <li>Shift active — green wave = "go score!"
     *   <li>Shift inactive — dim alliance color = "hold / pass"
     * </ul>
     */
    private void applyShiftPattern() {
        var shift = HubShiftUtil.getShiftedShiftInfo();

        if (shift.remainingTime() <= SHIFT_WARNING_SECS && shift.remainingTime() > 0) {
            // Approaching a shift boundary — fast warning strobe.
            strobe(Color.kOrange, STROBE_FAST_DURATION);
        } else if (shift.active()) {
            // Our shift is live — green wave signals "go score!"
            wave(Color.kGreen, Color.kBlack, WAVE_FAST_CYCLE_LENGTH, WAVE_FAST_DURATION);
        } else {
            // Not our shift — dim alliance color.
            setColor(dim(allianceColor, 0.25));
        }
    }

    private void clearState() {
        intaking = false;
        aimLock = false;
    }

    /** Return a dimmed copy of the given color (brightness 0.0–1.0). */
    private static Color dim(Color color, double brightness) {
        return new Color(color.red * brightness, color.green * brightness, color.blue * brightness);
    }

    private void setColor(Color color) {
        for (int i = 0; i < LEDsConstants.kNumLEDs; i++) {
            ledBuffer.setLED(i, color);
        }
    }

    private void wave(Color c1, Color c2, double cycleLength, double duration) {
        double x = (1 - ((Timer.getFPGATimestamp() % duration) / duration)) * 2.0 * Math.PI;
        double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
        for (int i = 0; i < LEDsConstants.kNumLEDs; i++) {
            x += xDiffPerLed;
            double ratio = (Math.pow(Math.sin(x), WAVE_EXPONENT) + 1.0) / 2.0;
            if (Double.isNaN(ratio)) {
                ratio = (-Math.pow(Math.sin(x + Math.PI), WAVE_EXPONENT) + 1.0) / 2.0;
            }
            if (Double.isNaN(ratio)) {
                ratio = 0.5;
            }
            double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
            double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
            double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
            ledBuffer.setLED(i, new Color(red, green, blue));
        }
    }

    private void strobe(Color color, double duration) {
        boolean on = ((Timer.getFPGATimestamp() % duration) / duration) > 0.5;
        setColor(on ? color : Color.kBlack);
    }

    // endregion
}
