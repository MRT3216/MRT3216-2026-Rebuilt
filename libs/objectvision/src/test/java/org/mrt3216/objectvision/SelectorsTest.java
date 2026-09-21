package org.mrt3216.objectvision;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;

class SelectorsTest {
    private static final Pose2d kRobotAtOrigin = new Pose2d(0, 0, Rotation2d.kZero);

    private static TrackedTarget track(double x, double y, int id) {
        return new TrackedTarget(new Translation2d(x, y), 1.0, 0.0, id);
    }

    // --- nearest() ---

    @Test
    void nearest_emptyList_returnsEmpty() {
        Optional<TrackedTarget> result = Selectors.nearest().select(List.of(), kRobotAtOrigin);
        assertTrue(result.isEmpty());
    }

    @Test
    void nearest_selectsClosest() {
        List<TrackedTarget> tracks = List.of(track(3.0, 0, 0), track(1.0, 0, 1), track(5.0, 0, 2));

        Optional<TrackedTarget> result = Selectors.nearest().select(tracks, kRobotAtOrigin);

        assertTrue(result.isPresent());
        assertEquals(1, result.get().trackId(), "Should select track at 1.0 m");
    }

    @Test
    void nearest_respectsRobotPosition() {
        Pose2d robotPose = new Pose2d(4.0, 0, Rotation2d.kZero);
        List<TrackedTarget> tracks = List.of(track(1.0, 0, 0), track(5.0, 0, 1));

        Optional<TrackedTarget> result = Selectors.nearest().select(tracks, robotPose);

        assertTrue(result.isPresent());
        assertEquals(1, result.get().trackId(), "Should select track nearest to robot at x=4");
    }

    // --- nearestInHeading() ---

    @Test
    void nearestInHeading_filtersOutsideArc() {
        // Robot at origin facing +X (heading = 0)
        // Track directly ahead at (2, 0) → bearing 0 → within ±45 deg
        // Track to the side at (0, 2) → bearing 90 deg → outside ±45 deg
        List<TrackedTarget> tracks = List.of(track(2.0, 0, 0), track(0, 2.0, 1));

        double halfAngle = Math.toRadians(45.0);
        Optional<TrackedTarget> result =
                Selectors.nearestInHeading(halfAngle).select(tracks, kRobotAtOrigin);

        assertTrue(result.isPresent());
        assertEquals(0, result.get().trackId(), "Should select the track within heading arc");
    }

    @Test
    void nearestInHeading_selectsNearestWithinArc() {
        // Both tracks within 45 deg arc, different distances
        List<TrackedTarget> tracks = List.of(track(3.0, 0.5, 0), track(1.5, 0.2, 1));

        double halfAngle = Math.toRadians(45.0);
        Optional<TrackedTarget> result =
                Selectors.nearestInHeading(halfAngle).select(tracks, kRobotAtOrigin);

        assertTrue(result.isPresent());
        assertEquals(1, result.get().trackId(), "Should select the nearer track");
    }

    @Test
    void nearestInHeading_emptyWhenNoneInArc() {
        // Track behind the robot
        List<TrackedTarget> tracks = List.of(track(-2.0, 0, 0));

        double halfAngle = Math.toRadians(45.0);
        Optional<TrackedTarget> result =
                Selectors.nearestInHeading(halfAngle).select(tracks, kRobotAtOrigin);

        assertTrue(result.isEmpty(), "Track behind robot should not be selected");
    }

    @Test
    void nearestInHeading_respectsRobotHeading() {
        // Robot facing left (+Y direction, heading = 90 deg)
        Pose2d robotFacingLeft = new Pose2d(0, 0, Rotation2d.fromDegrees(90));
        // Track to the left at (0, 2) → bearing 90 deg → within arc
        // Track ahead at (2, 0) → bearing 0 deg → outside 90±45 deg arc
        List<TrackedTarget> tracks = List.of(track(2.0, 0, 0), track(0, 2.0, 1));

        double halfAngle = Math.toRadians(45.0);
        Optional<TrackedTarget> result =
                Selectors.nearestInHeading(halfAngle).select(tracks, robotFacingLeft);

        assertTrue(result.isPresent());
        assertEquals(1, result.get().trackId(), "Should select the track in robot's heading");
    }

    // --- densestCluster() ---

    @Test
    void densestCluster_selectsFromDenseArea() {
        // Cluster of 3 near (5, 0), isolated track at (1, 0)
        List<TrackedTarget> tracks =
                List.of(track(1.0, 0, 0), track(5.0, 0, 1), track(5.1, 0.1, 2), track(4.9, -0.1, 3));

        Optional<TrackedTarget> result = Selectors.densestCluster(0.5).select(tracks, kRobotAtOrigin);

        assertTrue(result.isPresent());
        int selectedId = result.get().trackId();
        assertTrue(
                selectedId >= 1 && selectedId <= 3,
                "Should select from the dense cluster (ids 1-3), got " + selectedId);
    }

    @Test
    void densestCluster_singleTrack_returnsThat() {
        List<TrackedTarget> tracks = List.of(track(2.0, 1.0, 0));

        Optional<TrackedTarget> result = Selectors.densestCluster(1.0).select(tracks, kRobotAtOrigin);

        assertTrue(result.isPresent());
        assertEquals(0, result.get().trackId());
    }

    @Test
    void densestCluster_centroidSelection() {
        // Three tracks in a line: (0, 0), (0.2, 0), (0.4, 0)
        // With radius 0.5, all are in one cluster. Centroid = (0.2, 0) → track id 1.
        List<TrackedTarget> tracks = List.of(track(0, 0, 0), track(0.2, 0, 1), track(0.4, 0, 2));

        Optional<TrackedTarget> result = Selectors.densestCluster(0.5).select(tracks, kRobotAtOrigin);

        assertTrue(result.isPresent());
        assertEquals(1, result.get().trackId(), "Should select track nearest to centroid (0.2, 0)");
    }

    @Test
    void densestCluster_emptyList_returnsEmpty() {
        Optional<TrackedTarget> result =
                Selectors.densestCluster(1.0).select(List.of(), kRobotAtOrigin);
        assertTrue(result.isEmpty());
    }
}
