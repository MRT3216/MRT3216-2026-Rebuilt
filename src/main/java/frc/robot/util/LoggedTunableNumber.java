package frc.robot.util;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * Lightweight tunable number wrapper used by several teams. Falls back to a default when tuning is
 * disabled and publishes a Junction LoggedNetworkNumber when enabled.
 */
public class LoggedTunableNumber implements DoubleSupplier {
    private static final String tableKey = "/Tuning";

    private final String key;
    private boolean hasDefault = false;
    private double defaultValue;
    private LoggedNetworkNumber dashboardNumber;
    private Map<Integer, Double> lastHasChangedValues = new HashMap<>();
    private final boolean tuningMode;

    /**
     * Create a new LoggedTunableNumber
     *
     * @param dashboardKey Key on dashboard
     * @param isTuning Whether we're in tuning mode (create NT entry)
     */
    public LoggedTunableNumber(String dashboardKey, boolean isTuning) {
        this.key = tableKey + "/" + dashboardKey;
        this.tuningMode = isTuning;
    }

    /**
     * Create a new LoggedTunableNumber with the default value
     *
     * @param dashboardKey Key on dashboard
     * @param defaultValue Default value
     * @param isTuning Whether we're in tuning mode
     */
    public LoggedTunableNumber(String dashboardKey, double defaultValue, boolean isTuning) {
        this(dashboardKey, isTuning);
        initDefault(defaultValue);
    }

    /**
     * Set the default value of the number. The default value can only be set once.
     *
     * @param defaultValue The default value
     */
    public void initDefault(double defaultValue) {
        if (!hasDefault) {
            hasDefault = true;
            this.defaultValue = defaultValue;
            if (tuningMode) {
                dashboardNumber = new LoggedNetworkNumber(key, defaultValue);
            }
        }
    }

    /**
     * Get the current value, from dashboard if available and in tuning mode.
     *
     * <p>When tuning mode is active this returns the greater of the dashboard-published value and the
     * local mirror kept by {@link #set}, so that programmatic updates are visible immediately within
     * the same loop cycle (before {@code LoggedNetworkNumber.periodic()} refreshes its NT cache).
     *
     * @return The current value
     */
    public double get() {
        if (!hasDefault) {
            return 0.0;
        }
        if (!tuningMode) {
            return defaultValue;
        }
        // In tuning mode: if the dashboard has diverged from our local mirror (i.e. the
        // operator moved the slider), prefer the dashboard value and sync the mirror.
        double dashValue = dashboardNumber.get();
        if (dashValue != defaultValue) {
            defaultValue = dashValue;
        }
        return defaultValue;
    }

    /**
     * Programmatically set the value. When tuning mode is active this writes through to the
     * underlying {@link org.littletonrobotics.junction.networktables.LoggedNetworkNumber}, so
     * dashboard widgets will update on the next cycle. Also updates the local default so that
     * back-to-back {@link #get()} calls within the same loop cycle see the new value immediately
     * (before {@code LoggedNetworkNumber.periodic()} has had a chance to refresh its cache).
     *
     * @param value The new value
     */
    public void set(double value) {
        // Always keep defaultValue in sync so get() returns the new value immediately,
        // even before the NT cache is refreshed on the next periodic() call.
        defaultValue = value;
        if (tuningMode && dashboardNumber != null) {
            dashboardNumber.set(value);
        }
    }

    /**
     * Checks whether the number has changed since our last check
     *
     * @param id Unique identifier for the caller to avoid conflicts when shared between multiple
     *     objects. Recommended approach is to pass the result of "hashCode()"
     * @return True if the number has changed since the last time this method was called, false
     *     otherwise.
     */
    public boolean hasChanged(int id) {
        if (!tuningMode) return false;

        double currentValue = get();
        Double lastValue = lastHasChangedValues.get(id);
        if (lastValue == null || currentValue != lastValue) {
            lastHasChangedValues.put(id, currentValue);
            return true;
        }

        return false;
    }

    /**
     * Runs action if any of the tunableNumbers have changed
     *
     * @param id Unique identifier for the caller to avoid conflicts when shared between multiple
     *     objects. Recommended approach is to pass the result of "hashCode()"
     * @param action Callback to run when any of the tunable numbers have changed. Access tunable
     *     numbers in order inputted in method
     * @param tunableNumbers All tunable numbers to check
     */
    public static void ifChanged(
            int id, Consumer<double[]> action, LoggedTunableNumber... tunableNumbers) {
        if (Arrays.stream(tunableNumbers).anyMatch(tunableNumber -> tunableNumber.hasChanged(id))) {
            action.accept(Arrays.stream(tunableNumbers).mapToDouble(LoggedTunableNumber::get).toArray());
        }
    }

    /** Runs action if any of the tunableNumbers have changed */
    public static void ifChanged(int id, Runnable action, LoggedTunableNumber... tunableNumbers) {
        ifChanged(id, values -> action.run(), tunableNumbers);
    }

    @Override
    public double getAsDouble() {
        return get();
    }
}
