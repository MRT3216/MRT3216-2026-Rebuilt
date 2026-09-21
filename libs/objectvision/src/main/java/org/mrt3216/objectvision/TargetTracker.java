package org.mrt3216.objectvision;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class TargetTracker {
    private static final int kMaxTracks = 20;
    private static final double kSmoothingAlpha = 0.3;

    private final TargetConfig config;
    private final ArrayList<MutableTrack> tracks;
    private final List<TrackedTarget> publicView;
    private int nextTrackId = 0;

    // Preallocated scratch arrays to avoid per-update allocation
    private final boolean[] detectionMatched;
    private final boolean[] trackMatched;

    public TargetTracker(TargetConfig config) {
        this.config = config;
        this.tracks = new ArrayList<>(kMaxTracks);
        this.publicView = new ArrayList<>(kMaxTracks);
        this.detectionMatched = new boolean[128];
        this.trackMatched = new boolean[kMaxTracks];
    }

    /**
     * Update tracks with new field-frame detections.
     *
     * @param fieldPositions detected positions in the field frame
     * @param timestamp FPGA timestamp of the detections
     */
    public void update(List<Translation2d> fieldPositions, double timestamp) {
        // Clamp to preallocated array size (PhotonVision caps at 127 targets)
        int numDetections = Math.min(fieldPositions.size(), detectionMatched.length);
        int numTracks = tracks.size();

        // Reset match flags
        for (int i = 0; i < numDetections; i++) {
            detectionMatched[i] = false;
        }
        for (int i = 0; i < numTracks; i++) {
            trackMatched[i] = false;
        }

        // Nearest-neighbor association: for each detection, find the closest unmatched track
        double gate = config.associationGateMeters();
        double gateSq = gate * gate;

        for (int di = 0; di < numDetections; di++) {
            Translation2d det = fieldPositions.get(di);
            double bestDistSq = gateSq;
            int bestTrack = -1;

            for (int ti = 0; ti < numTracks; ti++) {
                if (trackMatched[ti]) {
                    continue;
                }
                double dx = det.getX() - tracks.get(ti).x;
                double dy = det.getY() - tracks.get(ti).y;
                double distSq = dx * dx + dy * dy;
                if (distSq < bestDistSq) {
                    bestDistSq = distSq;
                    bestTrack = ti;
                }
            }

            if (bestTrack >= 0) {
                // Matched — exponential smoothing
                MutableTrack track = tracks.get(bestTrack);
                track.x += kSmoothingAlpha * (det.getX() - track.x);
                track.y += kSmoothingAlpha * (det.getY() - track.y);
                track.confidence = track.confidence + kSmoothingAlpha * (1.0 - track.confidence);
                track.lastUpdateTimestamp = timestamp;
                trackMatched[bestTrack] = true;
                detectionMatched[di] = true;
            }
        }

        // Create new tracks for unmatched detections (up to cap)
        for (int di = 0; di < numDetections; di++) {
            if (detectionMatched[di]) {
                continue;
            }
            if (tracks.size() >= kMaxTracks) {
                break;
            }
            Translation2d det = fieldPositions.get(di);
            tracks.add(
                    new MutableTrack(det.getX(), det.getY(), kSmoothingAlpha, timestamp, nextTrackId++));
        }

        // Expire old tracks (iterate backwards to allow removal)
        double expiryHorizon = timestamp - config.expirySeconds();
        for (int i = tracks.size() - 1; i >= 0; i--) {
            if (tracks.get(i).lastUpdateTimestamp < expiryHorizon) {
                tracks.remove(i);
            }
        }

        rebuildPublicView();
    }

    /** Returns an unmodifiable snapshot of the current tracks. */
    public List<TrackedTarget> getTracks() {
        return Collections.unmodifiableList(publicView);
    }

    public void clear() {
        tracks.clear();
        publicView.clear();
    }

    private void rebuildPublicView() {
        publicView.clear();
        for (int i = 0; i < tracks.size(); i++) {
            MutableTrack t = tracks.get(i);
            publicView.add(
                    new TrackedTarget(
                            new Translation2d(t.x, t.y), t.confidence, t.lastUpdateTimestamp, t.trackId));
        }
    }

    private static final class MutableTrack {
        double x;
        double y;
        double confidence;
        double lastUpdateTimestamp;
        final int trackId;

        MutableTrack(double x, double y, double confidence, double lastUpdateTimestamp, int trackId) {
            this.x = x;
            this.y = y;
            this.confidence = confidence;
            this.lastUpdateTimestamp = lastUpdateTimestamp;
            this.trackId = trackId;
        }
    }
}
