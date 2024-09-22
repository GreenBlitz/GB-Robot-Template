package frc.robot.poseestimator.photonvision;

import frc.robot.poseestimator.observations.VisionObservation;
import frc.robot.poseestimator.photonvision.photoncamera.GBPhotonCamera;
import frc.robot.poseestimator.photonvision.photoncamera.PhotonTargetData;

import java.util.ArrayList;

public class PhotonVisionObservationFiltered {

	private ArrayList<GBPhotonCamera> cameras;

	public PhotonVisionObservationFiltered(String logPath) {
        for (CameraConfiguration cameraConfiguration : PhotonConstants.CAMERAS_CONFIGURATION) {
            cameras.add(new GBPhotonCamera(cameraConfiguration));
        }
    }

    private boolean isDataAmbiguous(PhotonTargetData targetData) {
        return targetData.ambiguity() >= PhotonConstants.MAXIMUM_AMBIGUOUS;
    }
    
    private boolean isDataLatencyTooHigh(PhotonTargetData targetData) {
        return targetData.latency() >= PhotonConstants.MAXIMUM_LATENCY;
    }
    
    private boolean keepLimelightData(PhotonTargetData targetData) {
        return !isDataAmbiguous(targetData) && !isDataLatencyTooHigh(targetData);
    }

}
