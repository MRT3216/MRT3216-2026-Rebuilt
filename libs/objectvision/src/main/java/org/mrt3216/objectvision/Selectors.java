package org.mrt3216.objectvision;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.Optional;

public final class Selectors {
    private Selectors() {}

    /** Selects the track closest to the robot. */
    public static TargetSelector nearest() {
        return (tracks, robotPose) -> {
            if (tracks.isEmpty()) {
                return Optional.empty();
            }
            Translation2d robotPos = robotPose.getTranslation();
            TrackedTarget best = null;
            double bestDistSq = Double.MAX_VALUE;
            for (int i = 0; i < tracks.size(); i++) {
                TrackedTarget t = tracks.get(i);
                double dx = t.position().getX() - robotPos.getX();
                double dy = t.position().getY() - robotPos.getY();
                double distSq = dx * dx + dy * dy;
                if (distSq < bestDistSq) {
                    bestDistSq = distSq;
                    best = t;
                }
            }
            return Optional.ofNullable(best);
        };
    }

    /**
     * Selects the nearest track within a heading arc centered on the robot's heading.
     *
     * @param halfAngleRadians half-width of the arc (radians)
     */
    public static TargetSelector nearestInHeading(double halfAngleRadians) {
        return (tracks, robotPose) -> {
            if (tracks.isEmpty()) {
                return Optional.empty();
            }
            Translation2d robotPos = robotPose.getTranslation();
            double robotHeading = robotPose.getRotation().getRadians();
            TrackedTarget best = null;
            double bestDistSq = Double.MAX_VALUE;

            for (int i = 0; i < tracks.size(); i++) {
                TrackedTarget t = tracks.get(i);
                double dx = t.position().getX() - robotPos.getX();
                double dy = t.position().getY() - robotPos.getY();

                double bearingToTarget = Math.atan2(dy, dx);
                double angleDiff = normalizeAngle(bearingToTarget - robotHeading);
                if (Math.abs(angleDiff) > halfAngleRadians) {
                    continue;
                }

                double distSq = dx * dx + dy * dy;
                if (distSq < bestDistSq) {
                    bestDistSq = distSq;
                    best = t;
                }
            }
            return Optional.ofNullable(best);
        };
    }

    /**
     * Selects the track nearest to the centroid of the densest cluster.
     *
     * <p>Density is measured as the count of tracks within {@code radiusMeters} of each track.
     *
     * @param radiusMeters cluster radius
     */
    public static TargetSelector densestCluster(double radiusMeters) {
        return (tracks, robotPose) -> {
            if (tracks.isEmpty()) {
                return Optional.empty();
            }
            double radiusSq = radiusMeters * radiusMeters;

            // Find the track with the most neighbors (including itself)
            int bestIdx = 0;
            int bestCount = 0;

            for (int i = 0; i < tracks.size(); i++) {
                Translation2d pos = tracks.get(i).position();
                int count = 0;
                for (int j = 0; j < tracks.size(); j++) {
                    Translation2d other = tracks.get(j).position();
                    double dx = pos.getX() - other.getX();
                    double dy = pos.getY() - other.getY();
                    if (dx * dx + dy * dy <= radiusSq) {
                        count++;
                    }
                }
                if (count > bestCount) {
                    bestCount = count;
                    bestIdx = i;
                }
            }

            // Compute centroid of the densest cluster
            Translation2d center = tracks.get(bestIdx).position();
            double cx = 0;
            double cy = 0;
            int clusterSize = 0;
            for (int j = 0; j < tracks.size(); j++) {
                Translation2d other = tracks.get(j).position();
                double dx = center.getX() - other.getX();
                double dy = center.getY() - other.getY();
                if (dx * dx + dy * dy <= radiusSq) {
                    cx += other.getX();
                    cy += other.getY();
                    clusterSize++;
                }
            }
            cx /= clusterSize;
            cy /= clusterSize;

            // Return track nearest to centroid
            TrackedTarget nearest = null;
            double nearestDistSq = Double.MAX_VALUE;
            for (int j = 0; j < tracks.size(); j++) {
                Translation2d other = tracks.get(j).position();
                double dx2 = cx - other.getX();
                double dy2 = cy - other.getY();
                double distSq = dx2 * dx2 + dy2 * dy2;
                if (distSq < nearestDistSq) {
                    nearestDistSq = distSq;
                    nearest = tracks.get(j);
                }
            }
            return Optional.ofNullable(nearest);
        };
    }

    /** Normalize an angle to [-pi, pi]. */
    private static double normalizeAngle(double angle) {
        while (angle > Math.PI) {
            angle -= 2.0 * Math.PI;
        }
        while (angle < -Math.PI) {
            angle += 2.0 * Math.PI;
        }
        return angle;
    }
}
