package frc.robot.poseestimator.photonvision.photonvisionfilters;

import frc.robot.poseestimator.observations.GameObjectPoseObservation;
import frc.robot.poseestimator.photonvision.CameraConfiguration;
import frc.robot.poseestimator.photonvision.PhotonVisionTargetRawData;
import frc.robot.poseestimator.photonvision.PhotonVisionTarget;
import java.util.ArrayList;
import java.util.List;

public class PhotonVisionGameObjectFilters extends PhotonVisionFiltered {

	public PhotonVisionGameObjectFilters(CameraConfiguration[] cameraConfigurations, PhotonVisionTarget target, String logPath) {
		super(cameraConfigurations, target, logPath);
	}

	public GameObjectPoseObservation getObservationFromRawData(PhotonVisionTargetRawData gameObjectData) {
		return new GameObjectPoseObservation(gameObjectData.targetPose(), gameObjectData.timestamp());
	}

	public List<GameObjectPoseObservation> getAllGameObjectPoses() {
		List<GameObjectPoseObservation> output = new ArrayList<>();
		for (PhotonVisionTargetRawData gameObjectData : getAllTargetData()) {
			if (keepPhotonVisionData(gameObjectData)) {
				output.add(getObservationFromRawData(gameObjectData));
			}
		}
		return output;
	}

	@Override
	protected boolean keepPhotonVisionData(PhotonVisionTargetRawData targetData) {
		return !isDataLatencyTooHigh(targetData) && !isDataTooAmbiguous(targetData);
	}

	@Override
	protected void subsystemPeriodic() {}

}
