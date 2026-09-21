package org.mrt3216.objectvision;

public record SimCameraModel(
        double hFovRadians,
        double vFovRadians,
        double maxRangeMeters,
        double yawNoiseStdDevRadians,
        double pitchNoiseStdDevRadians,
        double dropoutProbability,
        double latencySeconds,
        double frameRateHz) {}
