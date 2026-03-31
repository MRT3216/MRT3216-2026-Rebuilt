// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** IO implementation for physics sim using PhotonVision simulator. */
public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {
    private static VisionSystemSim visionSim;

    /**
     * Timestamp of the last {@code visionSim.update()} call. The sim only needs to be stepped once
     * per robot loop even though multiple camera instances share it. Each {@link #updateInputs} call
     * compares against this to avoid redundant (and expensive) updates.
     */
    private static double lastSimUpdateTimestamp = -1.0;

    private final Supplier<Pose2d> poseSupplier;
    private final PhotonCameraSim cameraSim;

    /**
     * Creates a new VisionIOPhotonVisionSim.
     *
     * @param name The name of the camera.
     * @param poseSupplier Supplier for the robot pose to use in simulation.
     */
    public VisionIOPhotonVisionSim(
            String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
        super(name, robotToCamera);
        this.poseSupplier = poseSupplier;

        // Initialize vision sim
        if (visionSim == null) {
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(aprilTagLayout);
        }

        // Add sim camera — use reduced resolution to keep the sim loop under 20ms.
        // Real cameras run 1600×1304; sim only needs enough fidelity for pose estimation.
        var cameraProperties = new SimCameraProperties();
        cameraProperties.setCalibration(320, 240, Rotation2d.fromDegrees(80));
        cameraSim = new PhotonCameraSim(camera, cameraProperties, aprilTagLayout);
        cameraSim.enableDrawWireframe(false);
        visionSim.addCamera(cameraSim, robotToCamera);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        // The VisionSystemSim is shared across all camera instances. Only step it
        // once per robot loop (guarded by timestamp) to avoid doing 4× the work.
        double now = Timer.getFPGATimestamp();
        if (now != lastSimUpdateTimestamp) {
            visionSim.update(poseSupplier.get());
            lastSimUpdateTimestamp = now;
        }
        super.updateInputs(inputs);
    }
}
