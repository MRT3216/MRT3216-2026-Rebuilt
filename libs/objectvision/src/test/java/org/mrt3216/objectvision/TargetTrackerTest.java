package org.mrt3216.objectvision;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class TargetTrackerTest {
    private static final TargetConfig kConfig =
            new TargetConfig(
                    "test", 0, // classId
                    0.075, // targetCenterHeightMeters
                    0.5, // minConfidence
                    5.0, // maxRangeMeters
                    1.0, // expirySeconds
                    0.5); // associationGateMeters

    private TargetTracker tracker;

    @BeforeEach
    void setUp() {
        tracker = new TargetTracker(kConfig);
    }

    @Test
    void singleDetection_createsTrack() {
        tracker.update(List.of(new Translation2d(1.0, 0.0)), 0.0);

        List<TrackedTarget> tracks = tracker.getTracks();
        assertEquals(1, tracks.size());
        assertEquals(1.0, tracks.get(0).position().getX(), 1e-6);
        assertEquals(0.0, tracks.get(0).position().getY(), 1e-6);
    }

    @Test
    void associationWithinGate_sameTrack() {
        tracker.update(List.of(new Translation2d(1.0, 0.0)), 0.0);
        int firstId = tracker.getTracks().get(0).trackId();

        // Detection 0.3 m away (within 0.5 m gate) → should associate
        tracker.update(List.of(new Translation2d(1.3, 0.0)), 0.1);

        List<TrackedTarget> tracks = tracker.getTracks();
        assertEquals(1, tracks.size(), "Should still be one track");
        assertEquals(firstId, tracks.get(0).trackId(), "Should be the same track ID");
    }

    @Test
    void detectionOutsideGate_newTrack() {
        tracker.update(List.of(new Translation2d(1.0, 0.0)), 0.0);

        // Detection 1.0 m away (outside 0.5 m gate) → new track
        tracker.update(List.of(new Translation2d(2.0, 0.0)), 0.1);

        List<TrackedTarget> tracks = tracker.getTracks();
        assertEquals(2, tracks.size(), "Should create a second track");
        assertNotEquals(tracks.get(0).trackId(), tracks.get(1).trackId());
    }

    @Test
    void expiry_removesOldTracks() {
        tracker.update(List.of(new Translation2d(1.0, 0.0)), 0.0);
        assertEquals(1, tracker.getTracks().size());

        // No detections, time passes beyond expiry (1.0 s)
        tracker.update(List.of(), 1.5);

        assertEquals(0, tracker.getTracks().size(), "Track should have expired");
    }

    @Test
    void expiry_keepsRecentTracks() {
        tracker.update(List.of(new Translation2d(1.0, 0.0)), 0.0);

        // Still within expiry window
        tracker.update(List.of(), 0.5);

        assertEquals(1, tracker.getTracks().size(), "Track should still be alive");
    }

    @Test
    void cap_limitsTrackCount() {
        // Create 25 tracks (beyond the 20 cap) by placing them far apart
        List<Translation2d> manyDetections = new java.util.ArrayList<>();
        for (int i = 0; i < 25; i++) {
            manyDetections.add(new Translation2d(i * 2.0, 0));
        }
        tracker.update(manyDetections, 0.0);

        assertTrue(
                tracker.getTracks().size() <= 20,
                "Track count should be capped at 20, got " + tracker.getTracks().size());
    }

    @Test
    void smoothing_converges() {
        // Start with a detection, then repeatedly detect slightly offset (within gate).
        // The smoothed position should converge toward the repeated detection.
        Translation2d target = new Translation2d(1.3, 0.2);
        tracker.update(List.of(new Translation2d(1.0, 0.0)), 0.0);

        for (int i = 1; i <= 30; i++) {
            tracker.update(List.of(target), i * 0.02);
        }

        List<TrackedTarget> tracks = tracker.getTracks();
        assertEquals(1, tracks.size(), "Should still be one track (within gate)");
        Translation2d smoothed = tracks.get(0).position();
        assertEquals(target.getX(), smoothed.getX(), 0.05, "X should converge to target");
        assertEquals(target.getY(), smoothed.getY(), 0.05, "Y should converge to target");
    }

    @Test
    void determinism_sameInputsSameOutput() {
        // Two trackers with the same input sequence should produce identical results
        TargetTracker tracker2 = new TargetTracker(kConfig);

        List<Translation2d> dets1 = List.of(new Translation2d(1.0, 0.5));
        List<Translation2d> dets2 = List.of(new Translation2d(1.1, 0.4), new Translation2d(3.0, 2.0));

        tracker.update(dets1, 0.0);
        tracker2.update(dets1, 0.0);
        tracker.update(dets2, 0.1);
        tracker2.update(dets2, 0.1);

        List<TrackedTarget> t1 = tracker.getTracks();
        List<TrackedTarget> t2 = tracker2.getTracks();

        assertEquals(t1.size(), t2.size());
        for (int i = 0; i < t1.size(); i++) {
            assertEquals(t1.get(i).position().getX(), t2.get(i).position().getX(), 1e-9);
            assertEquals(t1.get(i).position().getY(), t2.get(i).position().getY(), 1e-9);
            assertEquals(t1.get(i).trackId(), t2.get(i).trackId());
        }
    }

    @Test
    void clear_removesAllTracks() {
        tracker.update(List.of(new Translation2d(1.0, 0.0)), 0.0);
        assertEquals(1, tracker.getTracks().size());

        tracker.clear();
        assertEquals(0, tracker.getTracks().size());
    }

    @Test
    void multipleDetections_associateCorrectly() {
        // Create two distinct tracks
        tracker.update(List.of(new Translation2d(1.0, 0.0), new Translation2d(5.0, 0.0)), 0.0);
        assertEquals(2, tracker.getTracks().size());

        // Update both slightly within gate
        tracker.update(List.of(new Translation2d(1.1, 0.0), new Translation2d(5.1, 0.0)), 0.1);
        assertEquals(2, tracker.getTracks().size(), "Should still be two tracks");
    }
}
