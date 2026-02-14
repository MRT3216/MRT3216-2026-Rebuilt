package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.KickerSubsystem;
import frc.robot.subsystems.shooter.SpindexerSubsystem;

public class ShootCommands {
    public final KickerSubsystem kickerSubsystem;
    public final SpindexerSubsystem spindexerSubsystem;

    public ShootCommands(KickerSubsystem kickerSubsystem, SpindexerSubsystem spindexerSubsystem) {
        this.kickerSubsystem = kickerSubsystem;
        this.spindexerSubsystem = spindexerSubsystem;
    }

    public Command shoot() {
        return this.Clear()
                .withTimeout(Seconds.of(0.2))
                .andThen(spindexerSubsystem.setDutyCycle(0.0))
                .andThen(kickerSubsystem.setVelocity(RPM.of(100)))
                .withTimeout(Seconds.of(1.0))
                .andThen(spindexerSubsystem.setDutyCycle(0.5));
    }

    public Command Clear() {
        return kickerSubsystem
                .setVelocity(RPM.of(-.1))
                .alongWith(spindexerSubsystem.setDutyCycle(-0.1));
    }
}
