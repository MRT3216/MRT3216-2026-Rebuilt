package frc.robot.util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import java.util.function.Supplier;

/**
 * Helper utilities for configuring and working with CTRE Phoenix motor controllers.
 *
 * <p>Provides a small centralized helper for refreshing {@link StatusSignal}s and a retry helper
 * for Phoenix commands that may experience transient CAN errors during initialization.
 */
public final class PhoenixUtil {
    private PhoenixUtil() {}

    /** Refreshes the provided Phoenix {@link StatusSignal}s if any were provided. */
    public static void refresh(StatusSignal<?>... signals) {
        if (signals != null && signals.length > 0) {
            BaseStatusSignal.refreshAll(signals);
        }
    }

    /**
     * Attempts to run a Phoenix command repeatedly until it returns success or the maximum attempt
     * count is reached. Useful for flaky CAN configuration operations during initialization where
     * transient bus errors may occur.
     *
     * @param maxAttempts maximum attempts to try the command
     * @param command supplier that executes the Phoenix command and returns a {@link StatusCode}
     */
    public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
        for (int i = 0; i < maxAttempts; i++) {
            var error = command.get();
            if (error.isOK()) break;
        }
    }
}

// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.
