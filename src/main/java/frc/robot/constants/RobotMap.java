package frc.robot.constants;

/**
 * Mapping of hardware CAN IDs and other fixed device identifiers. Keep numeric IDs here to make it
 * easy to change wiring without searching the codebase.
 */
public final class RobotMap {
    private RobotMap() {}

    public static final class Shooter {
        private Shooter() {}

        public static final class Flywheel {
            public static final int kLeftMotorId = 51;
            public static final int kRightMotorId = 52;
        }

        public static final class Kicker {
            public static final int kMotorId = 56;
        }

        public static final class Turret {
            public static final int kMotorId = 54;
            // RoboRIO DIO channel for the PWM absolute encoder (10T pinion on 90T ring gear).
            // The other CRT encoder is the REV Through Bore plugged into the SparkMax's
            // absolute-encoder port (13T pinion) — no separate ID needed.
            public static final int kAbsoluteEncoderPwmChannel = 0;
        }

        public static final class Spindexer {
            public static final int kMotorId = 55;
        }

        public static final class Hood {
            public static final int kMotorId = 53;
        }
    }

    public static final class Intake {
        private Intake() {}

        public static final class Pivot {
            public static final int kLeftMotorId = 61;
            public static final int kRightMotorId = 62;
        }

        public static final class Roller {
            public static final int kMotorId = 60;
        }
    }

    public static final class LEDs {
        // TODO: Verify this matches the actual roboRIO PWM port the LED data line is
        // wired to.
        public static final int kPort = 0;
    }
}
