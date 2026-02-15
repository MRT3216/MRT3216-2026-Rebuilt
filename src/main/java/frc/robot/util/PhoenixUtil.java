// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.util;

import com.ctre.phoenix6.StatusCode;
import java.util.function.Supplier;

/** Helper utilities for configuring and working with Phoenix motor controllers. */
public class PhoenixUtil {
    /**
     * Attempts to run a Phoenix command repeatedly until it returns success or the maximum attempt
     * count is reached. Useful for flaky CAN configuration operations during initialization where
     * transient bus errors may occur.
     *
     * @param maxAttempts maximum attempts to try the command
     * @param command supplier that executes the Phoenix command and returns a {@link StatusCode}
     */
    public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
        // Repeatedly invoke the supplied Phoenix command until it reports OK or we exhaust
        // the attempt budget. We intentionally swallow transient failures here so higher-level
        // initialization can continue when the device eventually responds.
        for (int i = 0; i < maxAttempts; i++) {
            var error = command.get();
            if (error.isOK()) break;
        }
    }
}
