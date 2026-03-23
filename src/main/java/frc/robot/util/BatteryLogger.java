// Adapted from 6328 Mechanical Advantage — RobotCode2026Public
// https://github.com/Mechanical-Advantage/RobotCode2026Public
//
// Original source: org.littletonrobotics.frc2026.energy.BatteryLogger

package frc.robot.util;

import frc.robot.constants.Constants;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

/**
 * Tracks current, power, and energy consumption per subsystem and for the whole robot. Each
 * subsystem calls {@link #reportCurrentUsage(String, double...)} once per loop with its measured
 * current draw. After all subsystems have reported, {@link #periodicAfterScheduler()} is called
 * from {@code Robot.robotPeriodic()} to add fixed device currents, accumulate energy, and publish
 * everything to AdvantageKit.
 *
 * <p>Energy values are cumulative joules (watt-seconds) since boot — useful for estimating
 * remaining battery capacity.
 */
public class BatteryLogger {
    // Running totals across ALL subsystems + fixed devices.
    private double totalCurrent = 0.0;
    private double totalPower = 0.0;
    private double totalEnergy = 0.0;

    // Per-subsystem totals (keys may be hierarchical, e.g. "Drive/Module0-Drive").
    private final Map<String, Double> subsystemCurrents = new HashMap<>();
    private final Map<String, Double> subsystemPowers = new HashMap<>();
    private final Map<String, Double> subsystemEnergies = new HashMap<>();

    // Set by Robot.java each loop before periodicAfterScheduler() is called.
    private double batteryVoltage = 12.0;
    private double rioCurrent = 0.0;

    // region Setters (6328 uses lombok @Setter — we write them manually)

    public void setBatteryVoltage(double batteryVoltage) {
        this.batteryVoltage = batteryVoltage;
    }

    public void setRioCurrent(double rioCurrent) {
        this.rioCurrent = rioCurrent;
    }

    // endregion

    /**
     * Report the current draw of a subsystem for this loop cycle. Call once per subsystem per loop.
     * Multiple amp values are summed (useful for multi-motor mechanisms). Hierarchical keys (e.g.
     * "Drive/Module0-Drive") are aggregated into their parent ("Drive").
     *
     * @param key Subsystem identifier (e.g. "Hood", "Drive/Module0-Drive").
     * @param amps One or more current measurements in amps. Absolute value is used.
     */
    public void reportCurrentUsage(String key, double... amps) {
        double currentAmps = 0.0;
        for (double amp : amps) {
            currentAmps += Math.abs(amp);
        }
        double power = currentAmps * batteryVoltage;
        double energy = power * Constants.loopPeriodSecs;

        totalCurrent += currentAmps;
        totalPower += power;
        totalEnergy += energy;

        // Aggregate hierarchically: "Drive/Module0-Drive" aggregates into "Drive"
        // Also handles "-" separators like 6328's "Module0-Drive"
        String[] parts = key.split("[/\\-]");
        StringBuilder aggregateKey = new StringBuilder();
        for (int i = 0; i < parts.length; i++) {
            if (i > 0) {
                aggregateKey.append("/");
            }
            aggregateKey.append(parts[i]);
            String k = aggregateKey.toString();
            subsystemCurrents.merge(k, currentAmps, Double::sum);
            subsystemPowers.merge(k, power, Double::sum);
            subsystemEnergies.merge(k, energy, Double::sum);
        }
    }

    /**
     * Called from {@code Robot.robotPeriodic()} AFTER {@code CommandScheduler.getInstance().run()}.
     * Adds fixed device current draws, then logs everything via AdvantageKit.
     */
    public void periodicAfterScheduler() {
        // ── Fixed device currents ─────────────────────────────────────────────
        // These devices don't have per-device current sensing, so we use
        // manufacturer-specified typical draws (same values 6328 uses).
        reportCurrentUsage("FixedDevices/RoboRIO", rioCurrent);
        reportCurrentUsage("FixedDevices/CANcoders", 0.05 * 4); // 4 swerve CANcoders @ ~50mA each
        reportCurrentUsage("FixedDevices/Pigeon2", 0.04);
        reportCurrentUsage("FixedDevices/CANivore", 0.03);
        reportCurrentUsage("FixedDevices/Radio", 0.5);

        // ── Log totals ────────────────────────────────────────────────────────
        Logger.recordOutput("EnergyLogger/TotalCurrentAmps", totalCurrent);
        Logger.recordOutput("EnergyLogger/TotalPowerWatts", totalPower);
        Logger.recordOutput("EnergyLogger/TotalEnergyJoules", totalEnergy);

        // ── Log per-subsystem breakdowns ──────────────────────────────────────
        for (var entry : subsystemCurrents.entrySet()) {
            Logger.recordOutput("EnergyLogger/" + entry.getKey() + "/CurrentAmps", entry.getValue());
        }
        for (var entry : subsystemPowers.entrySet()) {
            Logger.recordOutput("EnergyLogger/" + entry.getKey() + "/PowerWatts", entry.getValue());
        }
        for (var entry : subsystemEnergies.entrySet()) {
            Logger.recordOutput("EnergyLogger/" + entry.getKey() + "/EnergyJoules", entry.getValue());
        }

        // Reset per-loop accumulators (energy is cumulative, so it persists in the maps).
        totalCurrent = 0.0;
        totalPower = 0.0;
        // Note: totalEnergy is NOT reset — it's a running total since boot.
        subsystemCurrents.clear();
        subsystemPowers.clear();
        // subsystemEnergies is NOT cleared — it accumulates across loops.
    }
}
