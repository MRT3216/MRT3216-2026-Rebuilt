package frc.robot.systems;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.KickerSubsystem;
import frc.robot.subsystems.shooter.SpindexerSubsystem;

/**
 * Aggregated shooter system (high-level grouping of shooter subsystems).
 *
 * <p>This class owns references to the flywheel, kicker, and spindexer subsystems and provides
 * higher-level commands that coordinate them (e.g. shooting routines).
 */
public class ShooterSystem {
    public final FlywheelSubsystem flywheel;
    public final KickerSubsystem kicker;
    public final SpindexerSubsystem spindexer;

    public ShooterSystem(
            FlywheelSubsystem flywheel, KickerSubsystem kicker, SpindexerSubsystem spindexer) {
        this.flywheel = flywheel;
        this.kicker = kicker;
        this.spindexer = spindexer;
    }

    /** High-level shoot command: clear, spin kicker, and feed spindexer. */
    public Command shoot() {
        return this.clear()
                .withTimeout(Seconds.of(0.2))
                .andThen(spindexer.setDutyCycle(0.0))
                .andThen(kicker.setVelocity(RPM.of(100)))
                .withTimeout(Seconds.of(1.0))
                .andThen(spindexer.setDutyCycle(0.5));
    }

    /** Clear the shooter (spin backwards and reverse spindexer briefly). */
    public Command clear() {
        return kicker.setVelocity(RPM.of(-0.1)).alongWith(spindexer.setDutyCycle(-0.1));
    }
}
