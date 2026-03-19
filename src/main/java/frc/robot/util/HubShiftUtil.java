// Adapted from Mechanical Advantage RobotCode2026Public
// https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/main/src/main/java/org/littletonrobotics/frc2026/util/HubShiftUtil.java
//
// Modifications for MRT3216:
//   - TOF bounds sourced from ShootingLookupTable (HUB mode) instead of LaunchCalculator,
//     so they stay correct automatically when ShooterLookupTables.java is updated.
//     Do NOT re-hardcode MIN_TOF / MAX_TOF here — edit ShooterLookupTables.java instead.
//   - Made ShiftInfo / ShiftEnum public for Robot.java logging
//   - Added FMS clock-sync and alliance-win override supplier

package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.shooter.ShootingLookupTable;
import java.util.Optional;
import java.util.function.Supplier;

/**
 * Tracks "hub shifts" — the alternating windows in the 2026 Rebuilt teleop period during which each
 * alliance may score into the hub.
 *
 * <p>Teleop (140 s) is split into six shifts. Shifts 1, 3, 5 belong to one alliance; 2, 4, 6 to the
 * other. Which alliance starts depends on the FMS game-specific message, or falls back to the
 * opposite of the robot's own alliance.
 *
 * <p>Call {@link #initialize()} once at the start of teleop.
 */
public class HubShiftUtil {

    /** Identifies the current shift period. */
    public enum ShiftEnum {
        TRANSITION,
        SHIFT1,
        SHIFT2,
        SHIFT3,
        SHIFT4,
        ENDGAME,
        AUTO,
        DISABLED
    }

    /**
     * Snapshot of the current hub-shift state.
     *
     * @param currentShift which shift window is active
     * @param elapsedTime seconds elapsed within the current shift window
     * @param remainingTime seconds remaining in the current shift window
     * @param active whether this alliance is allowed to score right now
     */
    public record ShiftInfo(
            ShiftEnum currentShift, double elapsedTime, double remainingTime, boolean active) {}

    // ── Timing constants ────────────────────────────────────────────────────────

    /** Total teleop period duration (seconds). */
    public static final double TELEOP_DURATION = 140.0;

    /** Auto period duration (seconds). */
    public static final double AUTO_END_TIME = 20.0;

    private static final double[] SHIFT_START = {0.0, 10.0, 35.0, 60.0, 85.0, 110.0};
    private static final double[] SHIFT_END = {10.0, 35.0, 60.0, 85.0, 110.0, 140.0};

    /** Alternating active/inactive schedules indexed by shift number. */
    private static final boolean[] ACTIVE_SCHEDULE = {true, true, false, true, false, true};

    private static final boolean[] INACTIVE_SCHEDULE = {true, false, true, false, true, true};

    // ── Time-of-flight fudge factors ────────────────────────────────────────────
    // Shift the "active" window slightly so the robot proactively starts/stops
    // shooting before each official boundary, accounting for ball travel time.

    private static final double MIN_FUEL_COUNT_DELAY = 1.0;
    private static final double MAX_FUEL_COUNT_DELAY = 2.0;
    private static final double SHIFT_END_EXTENSION = 3.0;

    // TOF bounds read directly from the HUB lookup table so they stay in sync when
    // ShooterLookupTables.java is updated — do not hardcode these values here.
    private static final ShootingLookupTable HUB_TABLE = new ShootingLookupTable(ShootingLookupTable.Mode.HUB);
    private static final double MIN_TOF = HUB_TABLE.getMinTimeOfFlight();
    private static final double MAX_TOF = HUB_TABLE.getMaxTimeOfFlight();

    private static final double APPROACHING_FUDGE = -1.0 * (MIN_TOF + MIN_FUEL_COUNT_DELAY);
    private static final double ENDING_FUDGE = SHIFT_END_EXTENSION - (MAX_TOF + MAX_FUEL_COUNT_DELAY);

    private static final Timer shiftTimer = new Timer();
    private static double shiftTimerOffset = 0.0;
    private static final double TIME_RESET_THRESHOLD = 3.0;

    private static Supplier<Optional<Boolean>> allianceWinOverride = () -> Optional.empty();

    private HubShiftUtil() {}

    // ── Public API ──────────────────────────────────────────────────────────────

    /**
     * Sets a supplier that overrides the FMS game-specific message for determining the first-active
     * alliance. {@code true} = we win (opponent goes first), {@code false} = they win (we go first).
     */
    public static void setAllianceWinOverride(Supplier<Optional<Boolean>> supplier) {
        allianceWinOverride = supplier;
    }

    /** Returns the current override value, if any. */
    public static Optional<Boolean> getAllianceWinOverride() {
        return allianceWinOverride.get();
    }

    /**
     * Determines which alliance is active first (i.e., starts scoring in SHIFT1). Uses the FMS
     * game-specific message; falls back to the opposite of our alliance by default.
     */
    public static Alliance getFirstActiveAlliance() {
        Alliance ds = DriverStation.getAlliance().orElse(Alliance.Blue);

        Optional<Boolean> override = getAllianceWinOverride();
        if (override.isPresent()) {
            // true = we won → opponent goes first; false = they won → we go first
            return override.get() ? (ds == Alliance.Blue ? Alliance.Red : Alliance.Blue) : ds;
        }

        String msg = DriverStation.getGameSpecificMessage();
        if (!msg.isEmpty()) {
            char c = msg.charAt(0);
            if (c == 'R') return Alliance.Blue;
            if (c == 'B') return Alliance.Red;
        }

        return ds == Alliance.Blue ? Alliance.Red : Alliance.Blue;
    }

    /** Resets and starts the internal shift timer. Call once at the beginning of teleop. */
    public static void initialize() {
        shiftTimerOffset = 0.0;
        shiftTimer.restart();
    }

    /**
     * Returns the official (unmodified) shift info — shift boundaries exactly as the game rules
     * define them.
     */
    public static ShiftInfo getOfficialShiftInfo() {
        return computeShiftInfo(getSchedule(), SHIFT_START, SHIFT_END);
    }

    /**
     * Returns the "shifted" shift info — boundaries nudged by TOF fudge factors so the robot starts
     * and stops shooting slightly before each official transition.
     */
    public static ShiftInfo getShiftedShiftInfo() {
        boolean[] sched = getSchedule();
        if (sched[1]) {
            // We are in the "starting active" configuration
            double[] starts = {
                0.0,
                10.0,
                35.0 + ENDING_FUDGE,
                60.0 + APPROACHING_FUDGE,
                85.0 + ENDING_FUDGE,
                110.0 + APPROACHING_FUDGE
            };
            double[] ends = {
                10.0,
                35.0 + ENDING_FUDGE,
                60.0 + APPROACHING_FUDGE,
                85.0 + ENDING_FUDGE,
                110.0 + APPROACHING_FUDGE,
                140.0
            };
            return computeShiftInfo(sched, starts, ends);
        }
        // "Starting inactive" configuration
        double[] starts = {
            0.0,
            10.0 + ENDING_FUDGE,
            35.0 + APPROACHING_FUDGE,
            60.0 + ENDING_FUDGE,
            85.0 + APPROACHING_FUDGE,
            110.0
        };
        double[] ends = {
            10.0 + ENDING_FUDGE,
            35.0 + APPROACHING_FUDGE,
            60.0 + ENDING_FUDGE,
            85.0 + APPROACHING_FUDGE,
            110.0,
            140.0
        };
        return computeShiftInfo(sched, starts, ends);
    }

    // ── Internal helpers ────────────────────────────────────────────────────────

    private static boolean[] getSchedule() {
        Alliance first = getFirstActiveAlliance();
        Alliance ds = DriverStation.getAlliance().orElse(Alliance.Blue);
        return first == ds ? ACTIVE_SCHEDULE : INACTIVE_SCHEDULE;
    }

    private static ShiftInfo computeShiftInfo(boolean[] schedule, double[] starts, double[] ends) {
        double raw = shiftTimer.get();
        double current = raw - shiftTimerOffset;

        if (DriverStation.isAutonomousEnabled()) {
            return new ShiftInfo(ShiftEnum.AUTO, current, AUTO_END_TIME - current, true);
        }

        if (DriverStation.isEnabled()) {
            // Sync internal timer with FMS match time to avoid drift.
            double fieldTime = TELEOP_DURATION - DriverStation.getMatchTime();
            if (Math.abs(fieldTime - current) >= TIME_RESET_THRESHOLD
                    && fieldTime <= 135.0
                    && DriverStation.isFMSAttached()) {
                shiftTimerOffset += current - fieldTime;
                current = raw - shiftTimerOffset;
            }

            // Find which shift we're in (default to last = endgame).
            int idx = starts.length - 1;
            for (int i = 0; i < starts.length; i++) {
                if (current >= starts[i] && current < ends[i]) {
                    idx = i;
                    break;
                }
            }

            double elapsed = current - starts[idx];
            double remaining = ends[idx] - current;

            // Merge consecutive shifts with the same active state.
            if (idx > 0 && schedule[idx] == schedule[idx - 1]) {
                elapsed = current - starts[idx - 1];
            }
            if (idx < ends.length - 1 && schedule[idx] == schedule[idx + 1]) {
                remaining = ends[idx + 1] - current;
            }

            // ShiftEnum: idx 0=TRANSITION, 1=SHIFT1, 2=SHIFT2, 3=SHIFT3, 4=SHIFT4, 5=ENDGAME
            ShiftEnum shift = ShiftEnum.values()[idx];
            return new ShiftInfo(shift, elapsed, remaining, schedule[idx]);
        }

        return new ShiftInfo(ShiftEnum.DISABLED, 0.0, 0.0, false);
    }
}
