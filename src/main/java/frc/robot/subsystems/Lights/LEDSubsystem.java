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
import java.util.Optional;
import java.util.function.BooleanSupplier;

public class LEDSubsystem extends SubsystemBase {
    private final AddressableLEDBuffer ledBuffer;
    private final AddressableLED led;
    private int rainbowFirstPixelHue = 0;
    private Optional<Alliance> alliance = Optional.empty();
    private Color allianceColor = Color.kTurquoise;
    private Color secondaryDisabledColor = Color.kDarkBlue;
    private static LEDSubsystem instance;

    private static final double waveExponent = 0.4;
    private static final double waveFastCycleLength = 25.0;
    private static final double waveFastDuration = 0.25;
    private static final double waveAllianceCycleLength = 15.0;
    private static final double waveAllianceDuration = 2.0;
    private static final double strobeFastDuration = 0.1;
    private static final double strobeSlowDuration = 0.2;
    private boolean hopperFull = false;
    private boolean intaking = false;
    private boolean climbing = false;
    private boolean aimLock = false;

    private LEDSubsystem() {
        led = new AddressableLED(LEDs.kPort);
        ledBuffer = new AddressableLEDBuffer(LEDsConstants.kNumLEDs);
        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();
    }

    // region Lifecycle

    @Override
    public void periodic() {
        // Select LED mode
        setColor(Color.kBlack); // Default to off

        // Update alliance color
        if (DriverStation.isFMSAttached()) {
            alliance = DriverStation.getAlliance();
            allianceColor =
                    alliance
                            .map(alliance -> alliance == Alliance.Blue ? Color.kDarkBlue : Color.kRed)
                            .orElse(Color.kTurquoise);
        }

        if (DriverStation.isDisabled()) {
            wave(Color.kBlue, Color.kDarkOrange, waveAllianceCycleLength, waveAllianceDuration);
        } else if (DriverStation.isAutonomous()) {
            wave(Color.kDarkOrange, Color.kDarkBlue, waveFastCycleLength, waveFastDuration);
        } else {
            // Teleop
            if (climbing) {
                rainbow();
            } else if (intaking) {
                strobe(Color.kPurple, strobeFastDuration);
            } else if (aimLock) {
                setColor(Color.kGreen);
            } else if (hopperFull) {
                wave(Color.kGreen, Color.kBlack, waveFastCycleLength, waveFastDuration);
            } else {
                setColor(allianceColor);
            }
        }

        // Update LEDs
        led.setData(ledBuffer);
    }

    // endregion

    // region Private helpers

    private void setRGBValue(int r, int g, int b) {
        for (int i = 0; i < LEDsConstants.kNumLEDs; i++) {
            ledBuffer.setRGB(i, r, g, b);
        }
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
            double ratio = (Math.pow(Math.sin(x), waveExponent) + 1.0) / 2.0;
            if (Double.isNaN(ratio)) {
                ratio = (-Math.pow(Math.sin(x + Math.PI), waveExponent) + 1.0) / 2.0;
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

    private void rainbow() {
        // For every pixel
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final var hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
            // Set the value
            ledBuffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        rainbowFirstPixelHue += 3;
        // Check bounds
        rainbowFirstPixelHue %= 180;
    }

    private void setOff() {
        setColor(Color.kBlack);
    }

    // endregion

    // region Public API

    public boolean hasNote() {
        return hopperFull;
    }

    public Command setHubLEDCommand(BooleanSupplier on) {
        return this.runOnce(
                () -> {
                    clearState();
                    aimLock = on.getAsBoolean();
                });
    }

    public Command setIntakingLEDCommand(BooleanSupplier on) {
        return this.runOnce(
                () -> {
                    clearState();
                    intaking = on.getAsBoolean();
                });
    }

    public Command setLEDCommand(BooleanSupplier on) {
        return this.runOnce(
                () -> {
                    clearState();
                    hopperFull = on.getAsBoolean();
                });
    }

    public Command setClimbingLEDCommand(BooleanSupplier on) {
        return this.runOnce(() -> climbing = on.getAsBoolean());
    }

    private void clearState() {
        intaking = false;
        hopperFull = false;
        aimLock = false;
        climbing = false;
    }

    // endregion

    // region Singleton

    public static LEDSubsystem getInstance() {
        if (instance == null) {
            // if instance is null, initialize
            instance = new LEDSubsystem();
        }
        return instance;
    }

    // endregion
}
