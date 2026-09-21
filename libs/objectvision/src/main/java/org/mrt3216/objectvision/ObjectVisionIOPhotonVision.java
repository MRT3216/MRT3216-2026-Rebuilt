package org.mrt3216.objectvision;

import java.util.ArrayList;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class ObjectVisionIOPhotonVision implements ObjectVisionIO {
    private final PhotonCamera camera;
    private final ArrayList<DetectionObservation> observations = new ArrayList<>();

    public ObjectVisionIOPhotonVision(String cameraName) {
        camera = new PhotonCamera(cameraName);
    }

    @Override
    public void updateInputs(ObjectVisionIOInputs inputs) {
        inputs.connected = camera.isConnected();

        observations.clear();
        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
            double timestamp = result.getTimestampSeconds();
            for (PhotonTrackedTarget target : result.getTargets()) {
                observations.add(
                        new DetectionObservation(
                                target.getDetectedObjectClassID(),
                                target.getDetectedObjectConfidence(),
                                Math.toRadians(target.getYaw()),
                                Math.toRadians(target.getPitch()),
                                target.getArea(),
                                timestamp));
            }
        }

        inputs.detections = new DetectionObservation[observations.size()];
        for (int i = 0; i < observations.size(); i++) {
            inputs.detections[i] = observations.get(i);
        }
    }
}
