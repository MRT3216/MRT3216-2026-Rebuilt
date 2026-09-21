package org.mrt3216.objectvision;

public record TargetConfig(
        String name,
        int classId,
        double targetCenterHeightMeters,
        double minConfidence,
        double maxRangeMeters,
        double expirySeconds,
        double associationGateMeters) {}
