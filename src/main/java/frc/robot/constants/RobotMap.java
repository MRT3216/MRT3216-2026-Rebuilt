package frc.robot.constants;

/**
 * Mapping of hardware CAN IDs and other fixed device identifiers. Keep numeric IDs here to make it
 * easy to change wiring without searching the codebase.
 */
public final class RobotMap {
    public final class Shooter {
        public final class Flywheel {
            public static final int kLeftMotorId = 51;
            public static final int kRightMotorId = 52;
        }

        public final class Kicker {
            public static final int kMotorId = 56;
        }

        public final class Turret {
            public static final int kMotorId = 53;
        }

        public final class Spindexer {
            public static final int kMotorId = 55;
        }

        public final class Hood {
            public static final int kMotorId = 53;
        }
    }

    public final class Intake {
        public final class Pivot {
            public static final int kLeftMotorId = 61;
            public static final int kRightMotorId = 62;
        }

        public final class Roller {
            public static final int kMotorId = 63;
        }
    }
}
