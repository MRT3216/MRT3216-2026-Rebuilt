// Adapted from https://github.com/hammerheads5000/FuelSim for MRT3216.
// Ported to SubsystemBase so the scheduler drives updateSim() automatically.
// Hub/field geometry updated to use MRT3216 FieldConstants where possible.
package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.util.shooter.HybridTurretUtil;
import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * Physics-based fuel (ball) simulation for the 2026 FRC game.
 *
 * <p>Simulates the trajectory, field collisions, hub scoring, and robot interactions for game
 * pieces ("fuel"). Publishes ball positions to NetworkTables for visualization in AdvantageScope.
 * Register the robot via {@link #registerRobot} and call {@link #launchFromShotSolution} when your
 * shooter fires to see simulated trajectories.
 *
 * <p>Extends {@link SubsystemBase} so {@link #periodic()} is called automatically by the
 * CommandScheduler every loop. Call {@link #start()} once (e.g., in sim init) to begin ticking.
 *
 * <p>Original FuelSim by hammerheads5000 (https://github.com/hammerheads5000/FuelSim).
 */
public class FuelSim extends SubsystemBase {
    // ── Physics constants ───────────────────────────────────────────────────────
    protected static final double PERIOD = 0.02; // sec
    protected static final Translation3d GRAVITY = new Translation3d(0, 0, -9.81); // m/s²
    // Room-temp dry air density: https://en.wikipedia.org/wiki/Density_of_air#Dry_air
    protected static final double AIR_DENSITY = 1.2041; // kg/m³
    protected static final double FIELD_COR =
            Math.sqrt(22.0 / 51.5); // field coefficient of restitution
    protected static final double FUEL_COR = 0.5; // fuel–fuel COR
    protected static final double NET_COR = 0.2; // fuel–net COR
    protected static final double ROBOT_COR = 0.1; // fuel–robot COR
    /** Fuel (ball) radius from FieldConstants: diameter 5.91 in → radius 0.075 m. */
    protected static final double FUEL_RADIUS = FieldConstants.fuelDiameter / 2.0; // m

    protected static final double FRICTION = 0.1; // fraction of horizontal vel lost per sec on ground
    protected static final double FUEL_MASS = 0.448 * 0.45392; // kg
    protected static final double FUEL_CROSS_AREA = Math.PI * FUEL_RADIUS * FUEL_RADIUS;
    // Drag coeff for smooth sphere: https://en.wikipedia.org/wiki/Drag_coefficient
    protected static final double DRAG_COF = 0.47;
    protected static final double DRAG_FORCE_FACTOR = 0.5 * AIR_DENSITY * DRAG_COF * FUEL_CROSS_AREA;

    // ── Field dimensions (from FieldConstants) ──────────────────────────────────
    protected static final double FIELD_LENGTH = FieldConstants.fieldLength; // m
    protected static final double FIELD_WIDTH = FieldConstants.fieldWidth; // m

    // ── Trench / bump geometry (2026 field spec, values in metres) ──────────────
    protected static final double TRENCH_WIDTH = 1.265;
    protected static final double TRENCH_BLOCK_WIDTH = 0.305;
    protected static final double TRENCH_HEIGHT = 0.565;
    protected static final double TRENCH_BAR_HEIGHT = 0.102;
    protected static final double TRENCH_BAR_WIDTH = 0.152;

    // ── XZ field-profile line segments (each segment spans a Y range) ───────────
    // These represent the ramp / trench cross-sections seen edge-on in the XZ plane.
    protected static final Translation3d[] FIELD_XZ_LINE_STARTS = {
        new Translation3d(0, 0, 0),
        new Translation3d(3.96, 1.57, 0),
        new Translation3d(3.96, FIELD_WIDTH / 2 + 0.60, 0),
        new Translation3d(4.61, 1.57, 0.165),
        new Translation3d(4.61, FIELD_WIDTH / 2 + 0.60, 0.165),
        new Translation3d(FIELD_LENGTH - 5.18, 1.57, 0),
        new Translation3d(FIELD_LENGTH - 5.18, FIELD_WIDTH / 2 + 0.60, 0),
        new Translation3d(FIELD_LENGTH - 4.61, 1.57, 0.165),
        new Translation3d(FIELD_LENGTH - 4.61, FIELD_WIDTH / 2 + 0.60, 0.165),
        new Translation3d(3.96, TRENCH_WIDTH, TRENCH_HEIGHT),
        new Translation3d(3.96, FIELD_WIDTH - 1.57, TRENCH_HEIGHT),
        new Translation3d(FIELD_LENGTH - 5.18, TRENCH_WIDTH, TRENCH_HEIGHT),
        new Translation3d(FIELD_LENGTH - 5.18, FIELD_WIDTH - 1.57, TRENCH_HEIGHT),
        new Translation3d(4.61 - TRENCH_BAR_WIDTH / 2, 0, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
        new Translation3d(
                4.61 - TRENCH_BAR_WIDTH / 2, FIELD_WIDTH - 1.57, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
        new Translation3d(
                FIELD_LENGTH - 4.61 - TRENCH_BAR_WIDTH / 2, 0, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
        new Translation3d(
                FIELD_LENGTH - 4.61 - TRENCH_BAR_WIDTH / 2,
                FIELD_WIDTH - 1.57,
                TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
    };

    protected static final Translation3d[] FIELD_XZ_LINE_ENDS = {
        new Translation3d(FIELD_LENGTH, FIELD_WIDTH, 0),
        new Translation3d(4.61, FIELD_WIDTH / 2 - 0.60, 0.165),
        new Translation3d(4.61, FIELD_WIDTH - 1.57, 0.165),
        new Translation3d(5.18, FIELD_WIDTH / 2 - 0.60, 0),
        new Translation3d(5.18, FIELD_WIDTH - 1.57, 0),
        new Translation3d(FIELD_LENGTH - 4.61, FIELD_WIDTH / 2 - 0.60, 0.165),
        new Translation3d(FIELD_LENGTH - 4.61, FIELD_WIDTH - 1.57, 0.165),
        new Translation3d(FIELD_LENGTH - 3.96, FIELD_WIDTH / 2 - 0.60, 0),
        new Translation3d(FIELD_LENGTH - 3.96, FIELD_WIDTH - 1.57, 0),
        new Translation3d(5.18, TRENCH_WIDTH + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT),
        new Translation3d(5.18, FIELD_WIDTH - 1.57 + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT),
        new Translation3d(FIELD_LENGTH - 3.96, TRENCH_WIDTH + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT),
        new Translation3d(FIELD_LENGTH - 3.96, FIELD_WIDTH - 1.57 + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT),
        new Translation3d(
                4.61 + TRENCH_BAR_WIDTH / 2,
                TRENCH_WIDTH + TRENCH_BLOCK_WIDTH,
                TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
        new Translation3d(4.61 + TRENCH_BAR_WIDTH / 2, FIELD_WIDTH, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
        new Translation3d(
                FIELD_LENGTH - 4.61 + TRENCH_BAR_WIDTH / 2,
                TRENCH_WIDTH + TRENCH_BLOCK_WIDTH,
                TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
        new Translation3d(
                FIELD_LENGTH - 4.61 + TRENCH_BAR_WIDTH / 2, FIELD_WIDTH, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
    };

    // ── Fuel particle ────────────────────────────────────────────────────────────

    protected static class Fuel {
        protected Translation3d pos;
        protected Translation3d vel;

        protected Fuel(Translation3d pos, Translation3d vel) {
            this.pos = pos;
            this.vel = vel;
        }

        protected Fuel(Translation3d pos) {
            this(pos, new Translation3d());
        }

        protected void update(boolean simulateAirResistance, int subticks) {
            pos = pos.plus(vel.times(PERIOD / subticks));
            if (pos.getZ() > FUEL_RADIUS) {
                Translation3d Fg = GRAVITY.times(FUEL_MASS);
                Translation3d Fd = new Translation3d();
                if (simulateAirResistance) {
                    double speed = vel.getNorm();
                    if (speed > 1e-6) {
                        Fd = vel.times(-DRAG_FORCE_FACTOR * speed);
                    }
                }
                Translation3d accel = Fg.plus(Fd).div(FUEL_MASS);
                vel = vel.plus(accel.times(PERIOD / subticks));
            }
            if (Math.abs(vel.getZ()) < 0.05 && pos.getZ() <= FUEL_RADIUS + 0.03) {
                vel = new Translation3d(vel.getX(), vel.getY(), 0);
                vel = vel.times(1 - FRICTION * PERIOD / subticks);
            }
            handleFieldCollisions(subticks);
        }

        protected void handleXZLineCollision(Translation3d lineStart, Translation3d lineEnd) {
            if (pos.getY() < lineStart.getY() || pos.getY() > lineEnd.getY()) return;
            Translation2d start2d = new Translation2d(lineStart.getX(), lineStart.getZ());
            Translation2d end2d = new Translation2d(lineEnd.getX(), lineEnd.getZ());
            Translation2d pos2d = new Translation2d(pos.getX(), pos.getZ());
            Translation2d lineVec = end2d.minus(start2d);
            Translation2d projected =
                    start2d.plus(lineVec.times(pos2d.minus(start2d).dot(lineVec) / lineVec.getSquaredNorm()));
            if (projected.getDistance(start2d) + projected.getDistance(end2d) > lineVec.getNorm()) return;
            double dist = pos2d.getDistance(projected);
            if (dist > FUEL_RADIUS) return;
            Translation3d normal =
                    new Translation3d(-lineVec.getY(), 0, lineVec.getX()).div(lineVec.getNorm());
            pos = pos.plus(normal.times(FUEL_RADIUS - dist));
            if (vel.dot(normal) > 0) return;
            vel = vel.minus(normal.times((1 + FIELD_COR) * vel.dot(normal)));
        }

        protected void handleFieldCollisions(int subticks) {
            for (int i = 0; i < FIELD_XZ_LINE_STARTS.length; i++) {
                handleXZLineCollision(FIELD_XZ_LINE_STARTS[i], FIELD_XZ_LINE_ENDS[i]);
            }
            // Field edges
            if (pos.getX() < FUEL_RADIUS && vel.getX() < 0) {
                pos = pos.plus(new Translation3d(FUEL_RADIUS - pos.getX(), 0, 0));
                vel = vel.plus(new Translation3d(-(1 + FIELD_COR) * vel.getX(), 0, 0));
            } else if (pos.getX() > FIELD_LENGTH - FUEL_RADIUS && vel.getX() > 0) {
                pos = pos.plus(new Translation3d(FIELD_LENGTH - FUEL_RADIUS - pos.getX(), 0, 0));
                vel = vel.plus(new Translation3d(-(1 + FIELD_COR) * vel.getX(), 0, 0));
            }
            if (pos.getY() < FUEL_RADIUS && vel.getY() < 0) {
                pos = pos.plus(new Translation3d(0, FUEL_RADIUS - pos.getY(), 0));
                vel = vel.plus(new Translation3d(0, -(1 + FIELD_COR) * vel.getY(), 0));
            } else if (pos.getY() > FIELD_WIDTH - FUEL_RADIUS && vel.getY() > 0) {
                pos = pos.plus(new Translation3d(0, FIELD_WIDTH - FUEL_RADIUS - pos.getY(), 0));
                vel = vel.plus(new Translation3d(0, -(1 + FIELD_COR) * vel.getY(), 0));
            }
            handleHubCollisions(Hub.BLUE_HUB, subticks);
            handleHubCollisions(Hub.RED_HUB, subticks);
            handleTrenchCollisions();
        }

        protected void handleHubCollisions(Hub hub, int subticks) {
            hub.handleHubInteraction(this, subticks);
            hub.fuelCollideSide(this);
            double netCollision = hub.fuelHitNet(this);
            if (netCollision != 0) {
                pos = pos.plus(new Translation3d(netCollision, 0, 0));
                vel = new Translation3d(-vel.getX() * NET_COR, vel.getY() * NET_COR, vel.getZ());
            }
        }

        protected void handleTrenchCollisions() {
            fuelCollideRectangle(
                    this,
                    new Translation3d(3.96, TRENCH_WIDTH, 0),
                    new Translation3d(5.18, TRENCH_WIDTH + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT));
            fuelCollideRectangle(
                    this,
                    new Translation3d(3.96, FIELD_WIDTH - 1.57, 0),
                    new Translation3d(5.18, FIELD_WIDTH - 1.57 + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT));
            fuelCollideRectangle(
                    this,
                    new Translation3d(FIELD_LENGTH - 5.18, TRENCH_WIDTH, 0),
                    new Translation3d(FIELD_LENGTH - 3.96, TRENCH_WIDTH + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT));
            fuelCollideRectangle(
                    this,
                    new Translation3d(FIELD_LENGTH - 5.18, FIELD_WIDTH - 1.57, 0),
                    new Translation3d(
                            FIELD_LENGTH - 3.96, FIELD_WIDTH - 1.57 + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT));
            fuelCollideRectangle(
                    this,
                    new Translation3d(4.61 - TRENCH_BAR_WIDTH / 2, 0, TRENCH_HEIGHT),
                    new Translation3d(
                            4.61 + TRENCH_BAR_WIDTH / 2,
                            TRENCH_WIDTH + TRENCH_BLOCK_WIDTH,
                            TRENCH_HEIGHT + TRENCH_BAR_HEIGHT));
            fuelCollideRectangle(
                    this,
                    new Translation3d(4.61 - TRENCH_BAR_WIDTH / 2, FIELD_WIDTH - 1.57, TRENCH_HEIGHT),
                    new Translation3d(
                            4.61 + TRENCH_BAR_WIDTH / 2, FIELD_WIDTH, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT));
            fuelCollideRectangle(
                    this,
                    new Translation3d(FIELD_LENGTH - 4.61 - TRENCH_BAR_WIDTH / 2, 0, TRENCH_HEIGHT),
                    new Translation3d(
                            FIELD_LENGTH - 4.61 + TRENCH_BAR_WIDTH / 2,
                            TRENCH_WIDTH + TRENCH_BLOCK_WIDTH,
                            TRENCH_HEIGHT + TRENCH_BAR_HEIGHT));
            fuelCollideRectangle(
                    this,
                    new Translation3d(
                            FIELD_LENGTH - 4.61 - TRENCH_BAR_WIDTH / 2, FIELD_WIDTH - 1.57, TRENCH_HEIGHT),
                    new Translation3d(
                            FIELD_LENGTH - 4.61 + TRENCH_BAR_WIDTH / 2,
                            FIELD_WIDTH,
                            TRENCH_HEIGHT + TRENCH_BAR_HEIGHT));
        }

        protected void addImpulse(Translation3d impulse) {
            vel = vel.plus(impulse);
        }
    }

    // ── Fuel–fuel collision (broadphase grid) ────────────────────────────────────

    protected static void handleFuelCollision(Fuel a, Fuel b) {
        Translation3d normal = a.pos.minus(b.pos);
        double distance = normal.getNorm();
        if (distance == 0) {
            normal = new Translation3d(1, 0, 0);
            distance = 1;
        }
        normal = normal.div(distance);
        double impulse = 0.5 * (1 + FUEL_COR) * (b.vel.minus(a.vel).dot(normal));
        double intersection = FUEL_RADIUS * 2 - distance;
        if (intersection <= 0) return;
        a.pos = a.pos.plus(normal.times(intersection / 2));
        b.pos = b.pos.minus(normal.times(intersection / 2));
        a.addImpulse(normal.times(impulse));
        b.addImpulse(normal.times(-impulse));
    }

    protected static final double CELL_SIZE = 0.25;
    protected static final int GRID_COLS = (int) Math.ceil(FIELD_LENGTH / CELL_SIZE);
    protected static final int GRID_ROWS = (int) Math.ceil(FIELD_WIDTH / CELL_SIZE);

    @SuppressWarnings("unchecked")
    protected final ArrayList<Fuel>[][] grid = new ArrayList[GRID_COLS][GRID_ROWS];

    private final ArrayList<ArrayList<Fuel>> activeCells = new ArrayList<>();

    protected void handleFuelCollisions(ArrayList<Fuel> fuels) {
        for (ArrayList<Fuel> cell : activeCells) cell.clear();
        activeCells.clear();

        for (Fuel fuel : fuels) {
            int col = (int) (fuel.pos.getX() / CELL_SIZE);
            int row = (int) (fuel.pos.getY() / CELL_SIZE);
            if (col >= 0 && col < GRID_COLS && row >= 0 && row < GRID_ROWS) {
                grid[col][row].add(fuel);
                if (grid[col][row].size() == 1) activeCells.add(grid[col][row]);
            }
        }

        for (Fuel fuel : fuels) {
            int col = (int) (fuel.pos.getX() / CELL_SIZE);
            int row = (int) (fuel.pos.getY() / CELL_SIZE);
            for (int i = col - 1; i <= col + 1; i++) {
                for (int j = row - 1; j <= row + 1; j++) {
                    if (i >= 0 && i < GRID_COLS && j >= 0 && j < GRID_ROWS) {
                        for (Fuel other : grid[i][j]) {
                            if (fuel != other
                                    && fuel.pos.getDistance(other.pos) < FUEL_RADIUS * 2
                                    && fuel.hashCode() < other.hashCode()) {
                                handleFuelCollision(fuel, other);
                            }
                        }
                    }
                }
            }
        }
    }

    // ── Instance state ───────────────────────────────────────────────────────────

    protected ArrayList<Fuel> fuels = new ArrayList<>();
    protected boolean running = false;
    protected boolean simulateAirResistance = false;
    protected Supplier<Pose2d> robotPoseSupplier = null;
    protected Supplier<ChassisSpeeds> robotFieldSpeedsSupplier = null;
    protected double robotWidth;
    protected double robotLength;
    protected double bumperHeight;
    protected ArrayList<SimIntake> intakes = new ArrayList<>();
    protected int subticks = 5;
    protected double loggingFreqHz = 10;
    protected Timer loggingTimer = new Timer();

    protected StructArrayPublisher<Translation3d> fuelPublisher;

    /**
     * Creates a new FuelSim that publishes fuel positions to the given NetworkTable key.
     *
     * @param tableKey NT path prefix (e.g., {@code "/FuelSim"})
     */
    public FuelSim(String tableKey) {
        for (int i = 0; i < GRID_COLS; i++)
            for (int j = 0; j < GRID_ROWS; j++) grid[i][j] = new ArrayList<>();

        fuelPublisher =
                NetworkTableInstance.getDefault()
                        .getStructArrayTopic(tableKey + "/Fuels", Translation3d.struct)
                        .publish();
    }

    /** Creates a FuelSim publishing to {@code "/Fuel Simulation"}. */
    public FuelSim() {
        this("/Fuel Simulation");
    }

    // ── SubsystemBase lifecycle ──────────────────────────────────────────────────

    /**
     * Called automatically by the CommandScheduler each loop. Advances the simulation when running
     * and throttled-logs fuel positions.
     */
    @Override
    public void periodic() {
        updateSim();
    }

    // ── Simulation control ───────────────────────────────────────────────────────

    /** Remove all fuel from the field. */
    public void clearFuel() {
        fuels.clear();
    }

    /**
     * Spawn the standard starting-fuel arrangement (neutral zone + depots). Call this after {@link
     * #start()} to pre-populate the field.
     */
    public void spawnStartingFuel() {
        Translation3d center = new Translation3d(FIELD_LENGTH / 2, FIELD_WIDTH / 2, FUEL_RADIUS);
        for (int i = 0; i < 15; i++) {
            for (int j = 0; j < 6; j++) {
                fuels.add(
                        new Fuel(
                                center.plus(new Translation3d(0.076 + 0.152 * j, 0.0254 + 0.076 + 0.152 * i, 0))));
                fuels.add(
                        new Fuel(
                                center.plus(new Translation3d(-0.076 - 0.152 * j, 0.0254 + 0.076 + 0.152 * i, 0))));
                fuels.add(
                        new Fuel(
                                center.plus(new Translation3d(0.076 + 0.152 * j, -0.0254 - 0.076 - 0.152 * i, 0))));
                fuels.add(
                        new Fuel(
                                center.plus(
                                        new Translation3d(-0.076 - 0.152 * j, -0.0254 - 0.076 - 0.152 * i, 0))));
            }
        }
        // Depots
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 4; j++) {
                fuels.add(
                        new Fuel(new Translation3d(0.076 + 0.152 * j, 5.95 + 0.076 + 0.152 * i, FUEL_RADIUS)));
                fuels.add(
                        new Fuel(new Translation3d(0.076 + 0.152 * j, 5.95 - 0.076 - 0.152 * i, FUEL_RADIUS)));
                fuels.add(
                        new Fuel(
                                new Translation3d(
                                        FIELD_LENGTH - 0.076 - 0.152 * j, 2.09 + 0.076 + 0.152 * i, FUEL_RADIUS)));
                fuels.add(
                        new Fuel(
                                new Translation3d(
                                        FIELD_LENGTH - 0.076 - 0.152 * j, 2.09 - 0.076 - 0.152 * i, FUEL_RADIUS)));
            }
        }
    }

    /** Publish current fuel positions to NetworkTables for AdvantageScope. */
    public void logFuels() {
        fuelPublisher.set(fuels.stream().map(f -> f.pos).toArray(Translation3d[]::new));
    }

    /** Start the simulation. {@link #periodic()} drives stepping each loop. */
    public void start() {
        running = true;
        loggingTimer.restart();
    }

    /** Pause the simulation. */
    public void stop() {
        running = false;
        loggingTimer.stop();
    }

    /** Enable aerodynamic drag force (slightly more realistic, slightly more CPU). */
    public void enableAirResistance() {
        simulateAirResistance = true;
    }

    /**
     * Set the number of physics sub-steps per 20 ms loop. Higher values are more accurate but slower.
     *
     * @param subticks physics steps per loop (default: 5)
     */
    public void setSubticks(int subticks) {
        this.subticks = subticks;
    }

    /**
     * Set the NT publish frequency (for AdvantageScope performance).
     *
     * @param loggingFreqHz update frequency in Hz (default: 10)
     */
    public void setLoggingFrequency(double loggingFreqHz) {
        this.loggingFreqHz = loggingFreqHz;
    }

    // ── Robot registration ───────────────────────────────────────────────────────

    /**
     * Register the robot for fuel collision and intake simulation.
     *
     * @param width robot width along Y axis (metres)
     * @param length robot length along X axis (metres)
     * @param bumperHeight top-of-bumper height from floor (metres)
     * @param poseSupplier robot pose supplier
     * @param fieldSpeedsSupplier field-relative chassis speeds supplier
     */
    public void registerRobot(
            double width,
            double length,
            double bumperHeight,
            Supplier<Pose2d> poseSupplier,
            Supplier<ChassisSpeeds> fieldSpeedsSupplier) {
        this.robotPoseSupplier = poseSupplier;
        this.robotFieldSpeedsSupplier = fieldSpeedsSupplier;
        this.robotWidth = width;
        this.robotLength = length;
        this.bumperHeight = bumperHeight;
    }

    /**
     * Register the robot for fuel collision and intake simulation (WPILib units overload).
     *
     * @param width robot width (Distance, Y axis)
     * @param length robot length (Distance, X axis)
     * @param bumperHeight top-of-bumper height from floor
     * @param poseSupplier robot pose supplier
     * @param fieldSpeedsSupplier field-relative chassis speeds supplier
     */
    public void registerRobot(
            Distance width,
            Distance length,
            Distance bumperHeight,
            Supplier<Pose2d> poseSupplier,
            Supplier<ChassisSpeeds> fieldSpeedsSupplier) {
        registerRobot(
                width.in(Meters),
                length.in(Meters),
                bumperHeight.in(Meters),
                poseSupplier,
                fieldSpeedsSupplier);
    }

    // ── Simulation step ──────────────────────────────────────────────────────────

    /**
     * Advance the simulation by one period (0.02 s). Called from {@link #periodic()}. Does nothing
     * when not running.
     */
    public void updateSim() {
        if (!running) return;
        stepSim();
    }

    /** Run one deterministic physics step regardless of the {@code running} flag. */
    public void stepSim() {
        for (int i = 0; i < subticks; i++) {
            for (Fuel fuel : fuels) fuel.update(simulateAirResistance, subticks);
            handleFuelCollisions(fuels);
            if (robotPoseSupplier != null) {
                handleRobotCollisions(fuels);
                handleIntakes(fuels);
            }
        }
        if (loggingTimer.advanceIfElapsed(1.0 / loggingFreqHz)) logFuels();
    }

    // ── Fuel spawning / launching ────────────────────────────────────────────────

    /**
     * Spawn a fuel at a given field position with a given velocity.
     *
     * @param pos spawn position (field frame, metres)
     * @param vel initial velocity vector (m/s)
     */
    public void spawnFuel(Translation3d pos, Translation3d vel) {
        fuels.add(new Fuel(pos, vel));
    }

    /**
     * Launch a fuel from the robot's current pose, accounting for robot velocity.
     *
     * @param launchVelocity ball exit speed
     * @param hoodAngle elevation angle from horizontal (0 = horizontal, PI/2 = straight up)
     * @param turretYaw robot-relative turret yaw angle
     * @param launchHeight height above ground of the launch point
     * @throws IllegalStateException if {@link #registerRobot} has not been called
     */
    public void launchFuel(
            LinearVelocity launchVelocity, Angle hoodAngle, Angle turretYaw, Distance launchHeight) {
        if (robotPoseSupplier == null || robotFieldSpeedsSupplier == null)
            throw new IllegalStateException("Robot must be registered before launching fuel.");

        Pose3d launchPose =
                new Pose3d(robotPoseSupplier.get())
                        .plus(
                                new Transform3d(
                                        new Translation3d(Meters.zero(), Meters.zero(), launchHeight),
                                        Rotation3d.kZero));
        ChassisSpeeds fieldSpeeds = robotFieldSpeedsSupplier.get();

        double elevation = hoodAngle.in(Radians);
        double horizontalVel = Math.cos(elevation) * launchVelocity.in(MetersPerSecond);
        double verticalVel = Math.sin(elevation) * launchVelocity.in(MetersPerSecond);
        double yawRad = turretYaw.plus(launchPose.getRotation().getMeasureZ()).in(Radians);
        double xVel = horizontalVel * Math.cos(yawRad) + fieldSpeeds.vxMetersPerSecond;
        double yVel = horizontalVel * Math.sin(yawRad) + fieldSpeeds.vyMetersPerSecond;

        spawnFuel(launchPose.getTranslation(), new Translation3d(xVel, yVel, verticalVel));
    }

    /**
     * Convenience: launch from a {@link HybridTurretUtil.ShotSolution}.
     *
     * <p>Converts the solution's flywheel speed to a ball exit velocity using the configured flywheel
     * radius and a 0.54 compression factor. Uses the turret height from {@link
     * ShooterConstants.TurretConstants#kTurretOffsetZ} as the launch height.
     *
     * @param shot the computed shot solution
     * @param hoodElevation elevation angle from horizontal (same convention as FuelSim)
     */
    public void launchFromShotSolution(HybridTurretUtil.ShotSolution shot, Angle hoodElevation) {
        if (robotPoseSupplier == null || robotFieldSpeedsSupplier == null) return;

        // Convert angular flywheel speed → ball exit linear velocity.
        // Factor 0.54 accounts for flywheel compression / slip (from FuelSim spec).
        double flywheelRadiusM = ShooterConstants.FlywheelConstants.kWheelDiameter.in(Meters) / 2.0;
        double exitSpeedMs = shot.flywheelSpeed().in(RadiansPerSecond) * flywheelRadiusM * 0.54;

        launchFuel(
                MetersPerSecond.of(exitSpeedMs),
                hoodElevation,
                shot.turretAzimuth(),
                ShooterConstants.TurretConstants.kTurretOffsetZ);
    }

    /**
     * Convenience: launch from a {@link TurretCalculator.ShotData}.
     *
     * <p>ShotData stores hoodAngle as the complement of elevation (0=vertical, PI/2=horizontal), so
     * this method converts it to the standard elevation convention used by FuelSim.
     *
     * @param shot the computed shot data
     * @param turretYaw robot-relative turret azimuth
     */
    public void launchFromShotData(TurretCalculator.ShotData shot, Angle turretYaw) {
        if (robotPoseSupplier == null || robotFieldSpeedsSupplier == null) return;

        // ShotData.hoodAngle is from vertical → convert to elevation from horizontal.
        double elevationRad = Math.PI / 2.0 - shot.getHoodAngle().in(Radians);

        launchFuel(
                shot.getLinearExitVelocity(),
                Radians.of(elevationRad),
                turretYaw,
                ShooterConstants.TurretConstants.kTurretOffsetZ);
    }

    // ── Robot collision handling ─────────────────────────────────────────────────

    protected void handleRobotCollision(Fuel fuel, Pose2d robot, Translation2d robotVel) {
        Translation2d relativePos =
                new Pose2d(fuel.pos.toTranslation2d(), Rotation2d.kZero).relativeTo(robot).getTranslation();
        if (fuel.pos.getZ() > bumperHeight) return;

        double distToBottom = -FUEL_RADIUS - robotLength / 2 - relativePos.getX();
        double distToTop = -FUEL_RADIUS - robotLength / 2 + relativePos.getX();
        double distToRight = -FUEL_RADIUS - robotWidth / 2 - relativePos.getY();
        double distToLeft = -FUEL_RADIUS - robotWidth / 2 + relativePos.getY();

        if (distToBottom > 0 || distToTop > 0 || distToRight > 0 || distToLeft > 0) return;

        Translation2d posOffset;
        if (distToBottom >= distToTop && distToBottom >= distToRight && distToBottom >= distToLeft)
            posOffset = new Translation2d(distToBottom, 0);
        else if (distToTop >= distToBottom && distToTop >= distToRight && distToTop >= distToLeft)
            posOffset = new Translation2d(-distToTop, 0);
        else if (distToRight >= distToBottom && distToRight >= distToTop && distToRight >= distToLeft)
            posOffset = new Translation2d(0, distToRight);
        else posOffset = new Translation2d(0, -distToLeft);

        posOffset = posOffset.rotateBy(robot.getRotation());
        fuel.pos = fuel.pos.plus(new Translation3d(posOffset));
        Translation2d normal = posOffset.div(posOffset.getNorm());
        if (fuel.vel.toTranslation2d().dot(normal) < 0)
            fuel.addImpulse(
                    new Translation3d(
                            normal.times(-fuel.vel.toTranslation2d().dot(normal) * (1 + ROBOT_COR))));
        if (robotVel.dot(normal) > 0)
            fuel.addImpulse(new Translation3d(normal.times(robotVel.dot(normal))));
    }

    protected void handleRobotCollisions(ArrayList<Fuel> fuels) {
        Pose2d robot = robotPoseSupplier.get();
        ChassisSpeeds speeds = robotFieldSpeedsSupplier.get();
        Translation2d robotVel = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        for (Fuel fuel : fuels) handleRobotCollision(fuel, robot, robotVel);
    }

    protected void handleIntakes(ArrayList<Fuel> fuels) {
        Pose2d robot = robotPoseSupplier.get();
        for (SimIntake intake : intakes) {
            for (int i = 0; i < fuels.size(); i++) {
                if (intake.shouldIntake(fuels.get(i), robot)) {
                    fuels.remove(i--);
                }
            }
        }
    }

    // ── Rectangle collision helper ───────────────────────────────────────────────

    protected static void fuelCollideRectangle(Fuel fuel, Translation3d start, Translation3d end) {
        if (fuel.pos.getZ() > end.getZ() + FUEL_RADIUS || fuel.pos.getZ() < start.getZ() - FUEL_RADIUS)
            return;
        double distLeft = start.getX() - FUEL_RADIUS - fuel.pos.getX();
        double distRight = fuel.pos.getX() - end.getX() - FUEL_RADIUS;
        double distTop = fuel.pos.getY() - end.getY() - FUEL_RADIUS;
        double distBottom = start.getY() - FUEL_RADIUS - fuel.pos.getY();
        if (distLeft > 0 || distRight > 0 || distTop > 0 || distBottom > 0) return;

        Translation2d collision;
        if (fuel.pos.getX() < start.getX()
                || (distLeft >= distRight && distLeft >= distTop && distLeft >= distBottom))
            collision = new Translation2d(distLeft, 0);
        else if (fuel.pos.getX() >= end.getX()
                || (distRight >= distLeft && distRight >= distTop && distRight >= distBottom))
            collision = new Translation2d(-distRight, 0);
        else if (fuel.pos.getY() > end.getY()
                || (distTop >= distLeft && distTop >= distRight && distTop >= distBottom))
            collision = new Translation2d(0, -distTop);
        else collision = new Translation2d(0, distBottom);

        if (collision.getX() != 0) {
            fuel.pos = fuel.pos.plus(new Translation3d(collision));
            fuel.vel = fuel.vel.plus(new Translation3d(-(1 + FIELD_COR) * fuel.vel.getX(), 0, 0));
        } else if (collision.getY() != 0) {
            fuel.pos = fuel.pos.plus(new Translation3d(collision));
            fuel.vel = fuel.vel.plus(new Translation3d(0, -(1 + FIELD_COR) * fuel.vel.getY(), 0));
        }
    }

    // ── Intake registration ──────────────────────────────────────────────────────

    /**
     * Register an intake region that removes fuel from the field. Coordinates are robot-relative
     * (metres, same axis conventions as robot frame).
     *
     * @param xMin front/back min (robot frame)
     * @param xMax front/back max (robot frame)
     * @param yMin left/right min (robot frame)
     * @param yMax left/right max (robot frame)
     * @param ableToIntake whether the intake is currently active
     * @param intakeCallback called each time a fuel is collected
     */
    public void registerIntake(
            double xMin,
            double xMax,
            double yMin,
            double yMax,
            BooleanSupplier ableToIntake,
            Runnable intakeCallback) {
        intakes.add(new SimIntake(xMin, xMax, yMin, yMax, ableToIntake, intakeCallback));
    }

    public void registerIntake(
            double xMin, double xMax, double yMin, double yMax, BooleanSupplier ableToIntake) {
        registerIntake(xMin, xMax, yMin, yMax, ableToIntake, () -> {});
    }

    public void registerIntake(
            double xMin, double xMax, double yMin, double yMax, Runnable intakeCallback) {
        registerIntake(xMin, xMax, yMin, yMax, () -> true, intakeCallback);
    }

    public void registerIntake(double xMin, double xMax, double yMin, double yMax) {
        registerIntake(xMin, xMax, yMin, yMax, () -> true, () -> {});
    }

    public void registerIntake(
            Distance xMin,
            Distance xMax,
            Distance yMin,
            Distance yMax,
            BooleanSupplier ableToIntake,
            Runnable intakeCallback) {
        registerIntake(
                xMin.in(Meters),
                xMax.in(Meters),
                yMin.in(Meters),
                yMax.in(Meters),
                ableToIntake,
                intakeCallback);
    }

    public void registerIntake(
            Distance xMin, Distance xMax, Distance yMin, Distance yMax, BooleanSupplier ableToIntake) {
        registerIntake(
                xMin.in(Meters), xMax.in(Meters), yMin.in(Meters), yMax.in(Meters), ableToIntake);
    }

    public void registerIntake(
            Distance xMin, Distance xMax, Distance yMin, Distance yMax, Runnable intakeCallback) {
        registerIntake(
                xMin.in(Meters), xMax.in(Meters), yMin.in(Meters), yMax.in(Meters), intakeCallback);
    }

    public void registerIntake(Distance xMin, Distance xMax, Distance yMin, Distance yMax) {
        registerIntake(xMin.in(Meters), xMax.in(Meters), yMin.in(Meters), yMax.in(Meters));
    }

    // ── Hub ──────────────────────────────────────────────────────────────────────

    /**
     * Represents a scoring hub on the field. Centers are derived from {@link FieldConstants#HUB_BLUE}
     * / {@link FieldConstants#HUB_RED}.
     */
    public static class Hub {
        /**
         * Blue alliance hub. Center XY from {@link FieldConstants#HUB_BLUE}. Exit position is the
         * reject port (where scored fuel comes out).
         */
        public static final Hub BLUE_HUB =
                new Hub(
                        FieldConstants.HUB_BLUE.toTranslation2d(),
                        new Translation3d(FieldConstants.HUB_BLUE.getX() + 0.69, FIELD_WIDTH / 2, 0.89),
                        1);

        /** Red alliance hub. Center XY from {@link FieldConstants#HUB_RED}. */
        public static final Hub RED_HUB =
                new Hub(
                        FieldConstants.HUB_RED.toTranslation2d(),
                        new Translation3d(FieldConstants.HUB_RED.getX() - 0.69, FIELD_WIDTH / 2, 0.89),
                        -1);

        /** Height of the hub funnel entrance (top of hub = 72 in = 1.829 m). */
        protected static final double ENTRY_HEIGHT = 1.829;
        /**
         * Scoring radius: funnel opening radius from {@link FieldConstants#FUNNEL_RADIUS} (24 in ≈ 0.61
         * m).
         */
        protected static final double ENTRY_RADIUS = FieldConstants.FUNNEL_RADIUS.in(Meters);

        /** Physical side length of the square hub base (47 in ≈ 1.194 m). */
        protected static final double SIDE = FieldConstants.Hub.width;

        protected static final double NET_HEIGHT_MAX = 3.057;
        protected static final double NET_HEIGHT_MIN = 1.5;
        protected static final double NET_OFFSET = SIDE / 2 + 0.261;
        protected static final double NET_WIDTH = 1.484;

        protected final Translation2d center;
        protected final Translation3d exit;
        protected final int exitVelXMult;
        protected int score = 0;

        protected Hub(Translation2d center, Translation3d exit, int exitVelXMult) {
            this.center = center;
            this.exit = exit;
            this.exitVelXMult = exitVelXMult;
        }

        protected void handleHubInteraction(Fuel fuel, int subticks) {
            if (didFuelScore(fuel, subticks)) {
                fuel.pos = exit;
                fuel.vel = getDispersalVelocity();
                score++;
            }
        }

        protected boolean didFuelScore(Fuel fuel, int subticks) {
            return fuel.pos.toTranslation2d().getDistance(center) <= ENTRY_RADIUS
                    && fuel.pos.getZ() <= ENTRY_HEIGHT
                    && fuel.pos.minus(fuel.vel.times(PERIOD / subticks)).getZ() > ENTRY_HEIGHT;
        }

        protected Translation3d getDispersalVelocity() {
            return new Translation3d(
                    exitVelXMult * (Math.random() + 0.1) * 1.5, Math.random() * 2 - 1, 0);
        }

        /** Reset this hub's score count. */
        public void resetScore() {
            score = 0;
        }

        /** Get the number of fuel balls scored in this hub since the last reset. */
        public int getScore() {
            return score;
        }

        protected void fuelCollideSide(Fuel fuel) {
            fuelCollideRectangle(
                    fuel,
                    new Translation3d(center.getX() - SIDE / 2, center.getY() - SIDE / 2, 0),
                    new Translation3d(
                            center.getX() + SIDE / 2, center.getY() + SIDE / 2, ENTRY_HEIGHT - 0.1));
        }

        protected double fuelHitNet(Fuel fuel) {
            if (fuel.pos.getZ() > NET_HEIGHT_MAX || fuel.pos.getZ() < NET_HEIGHT_MIN) return 0;
            if (fuel.pos.getY() > center.getY() + NET_WIDTH / 2
                    || fuel.pos.getY() < center.getY() - NET_WIDTH / 2) return 0;
            if (fuel.pos.getX() > center.getX() + NET_OFFSET * exitVelXMult)
                return Math.max(
                        0, center.getX() + NET_OFFSET * exitVelXMult - (fuel.pos.getX() - FUEL_RADIUS));
            else
                return Math.min(
                        0, center.getX() + NET_OFFSET * exitVelXMult - (fuel.pos.getX() + FUEL_RADIUS));
        }
    }

    // ── SimIntake (inner) ────────────────────────────────────────────────────────

    protected class SimIntake {
        double xMin, xMax, yMin, yMax;
        BooleanSupplier ableToIntake;
        Runnable callback;

        protected SimIntake(
                double xMin,
                double xMax,
                double yMin,
                double yMax,
                BooleanSupplier ableToIntake,
                Runnable intakeCallback) {
            this.xMin = xMin;
            this.xMax = xMax;
            this.yMin = yMin;
            this.yMax = yMax;
            this.ableToIntake = ableToIntake;
            this.callback = intakeCallback;
        }

        protected boolean shouldIntake(Fuel fuel, Pose2d robotPose) {
            if (!ableToIntake.getAsBoolean() || fuel.pos.getZ() > bumperHeight) return false;
            Translation2d rel =
                    new Pose2d(fuel.pos.toTranslation2d(), Rotation2d.kZero)
                            .relativeTo(robotPose)
                            .getTranslation();
            boolean result =
                    rel.getX() >= xMin && rel.getX() <= xMax && rel.getY() >= yMin && rel.getY() <= yMax;
            if (result) callback.run();
            return result;
        }
    }
}
