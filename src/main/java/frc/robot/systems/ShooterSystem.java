package frc.robot.systems;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.KickerSubsystem;
import frc.robot.subsystems.shooter.SpindexerSubsystem;
import frc.robot.subsystems.shooter.TurretSubsystem;
import frc.robot.util.HybridTurretUtil;
import frc.robot.util.ShootingLookupTable;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

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
    public final TurretSubsystem turret;
    public final HoodSubsystem hood;

    public ShooterSystem(
            FlywheelSubsystem flywheel,
            KickerSubsystem kicker,
            SpindexerSubsystem spindexer,
            TurretSubsystem turret,
            HoodSubsystem hood) {
        this.flywheel = flywheel;
        this.kicker = kicker;
        this.spindexer = spindexer;
        this.turret = turret;
        this.hood = hood;
    }

    /** High-level shoot command: clear, spin kicker, and feed spindexer. */
    public Command shoot() {
        // Targets — chosen as safe defaults for testing. Adjust as needed.
        final AngularVelocity flywheelTarget = Constants.ShooterConstants.kTargetFlywheel;

        // Start spinning the flywheel to target velocity
        Command spin = flywheel.setVelocity(flywheelTarget);

        // Run the clear routine concurrently while the flywheel is spinning up. Use a timed
        // version so the clear routine doesn't persist longer than intended.
        Command clearTimed =
                clear().withTimeout(Seconds.of(Constants.ShooterConstants.kClearDurationSecs));

        // Run spin and clear in parallel while waiting for the flywheel to reach speed
        Command spinAndClear = spin.alongWith(clearTimed);

        // When at speed, run kicker and spindexer to feed balls continuously until the user
        // stops the overall shoot command. The feed Core is a long-running command (no timeout).
        Command feedCore =
                kicker
                        .setVelocity(Constants.KickerConstants.kTargetVelocity)
                        .alongWith(spindexer.setVelocity(Constants.SpindexerConstants.kTargetVelocity));

        // Monitor command: lambda-based monitor that starts feeding once the flywheel first
        // reaches speed and keeps feeding until the user cancels the overall shoot command.
        final java.util.concurrent.atomic.AtomicBoolean feedStarted =
                new java.util.concurrent.atomic.AtomicBoolean(false);
        Command monitor =
                Commands.runEnd(
                        () -> {
                            if (!feedStarted.get() && flywheel.atSpeed().getAsBoolean()) {
                                if (!feedCore.isScheduled()) {
                                    CommandScheduler.getInstance().schedule(feedCore);
                                }
                                feedStarted.set(true);
                            }
                        },
                        () -> {
                            if (feedCore.isScheduled()) {
                                CommandScheduler.getInstance().cancel(feedCore);
                            }
                        },
                        kicker,
                        spindexer);

        // Compose: run spin+clear and the monitor together; feeding will start/stop while this
        // command is active based on the flywheel trigger. The whole command runs until the
        // user cancels it.
        return new edu.wpi.first.wpilibj2.command.ParallelCommandGroup(spinAndClear, monitor);
    }

    /** Clear the shooter (spin backwards and reverse spindexer briefly). */
    public Command clear() {
        // Use small negative closed-loop velocities to clear any jammed balls. Closed-loop
        // ensures repeatable behavior across real and sim.
        return kicker
                .setVelocity(Constants.KickerConstants.kClearVelocity)
                .alongWith(spindexer.setVelocity(Constants.SpindexerConstants.kClearVelocity));
    }

    /**
     * Dynamically aim turret and hood using the provided robot pose, chassis speeds, and target.
     * Returns a command that computes a ShotSolution, moves turret & hood, spins the flywheel, and
     * feeds when at speed.
     */
    public Command aimAndShoot(
            Supplier<Pose2d> robotPose,
            Supplier<ChassisSpeeds> fieldSpeeds,
            Supplier<Translation3d> targetSupplier,
            int refinementIterations,
            ShootingLookupTable.Mode tableMode) {
        AtomicReference<HybridTurretUtil.ShotSolution> shotRef = new AtomicReference<>();

        ShootingLookupTable table = new ShootingLookupTable(tableMode);

        // Continuously compute the shot solution and apply the computed flywheel setpoint
        var computeLoop =
                new edu.wpi.first.wpilibj2.command.RunCommand(
                        () -> {
                            try {
                                HybridTurretUtil.ShotSolution sol =
                                        HybridTurretUtil.computeMovingShot(
                                                robotPose.get(),
                                                fieldSpeeds.get(),
                                                targetSupplier.get(),
                                                refinementIterations,
                                                table);
                                shotRef.set(sol);
                                // Apply flywheel setpoint each loop so it updates as the solution
                                if (sol != null) {
                                    flywheel.setVelocity(sol.flywheelSpeed());
                                }
                            } catch (Exception ex) {
                                // Defensive: don't let an exception kill the loop; leave last solution
                            }
                        },
                        flywheel);

        // Supplier-backed commands that read the last computed solution (with null-safety)
        Command aimTurret =
                turret.setAngle(
                        () -> {
                            var s = shotRef.get();
                            return s != null ? s.turretAzimuth() : turret.getPosition();
                        });
        Command aimHood =
                hood.setAngle(
                        () -> {
                            var s = shotRef.get();
                            return s != null ? s.hoodAngle() : hood.getPosition();
                        });

        // Condition: shot computed and flywheel has reached configured target within error margin
        // Lambda-based wait: implement as a RunCommand that cancels itself when the condition
        // becomes true. Use an AtomicReference to capture the command instance for self-cancel.
        final java.util.concurrent.atomic.AtomicReference<Command> waitRef =
                new java.util.concurrent.atomic.AtomicReference<>();
        Command waitForSpin =
                Commands.run(
                                () -> {
                                    var s = shotRef.get();
                                    if (s != null && s.isValid() && flywheel.atSetpoint().getAsBoolean()) {
                                        var c = waitRef.get();
                                        if (c != null) {
                                            CommandScheduler.getInstance().cancel(c);
                                        }
                                    }
                                },
                                flywheel)
                        .withTimeout(Seconds.of(3));
        waitRef.set(waitForSpin);

        // Feed using kicker and spindexer at the computed shooter-speed-derived rates (applied once)
        Command feed =
                new InstantCommand(
                                () -> {
                                    var s = shotRef.get();
                                    if (s != null) {
                                        kicker.setVelocity(s.flywheelSpeed());
                                        spindexer.setVelocity(s.flywheelSpeed());
                                    }
                                },
                                kicker,
                                spindexer)
                        .withTimeout(Seconds.of(2));

        // Run the compute+aim loop until the flywheel is at speed (waitForSpin finishes), then feed
        Command computeAndAimRace =
                new edu.wpi.first.wpilibj2.command.ParallelRaceGroup(
                        computeLoop, aimTurret, aimHood, waitForSpin);

        return computeAndAimRace.andThen(feed);
    }
}
