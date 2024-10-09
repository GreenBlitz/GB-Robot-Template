package frc.robot.vision.photonvision.photonvisionfilters;

import frc.robot.poseestimator.observations.GameObjectPoseObservation;
import frc.robot.vision.photonvision.CameraConfiguration;
import frc.robot.vision.photonvision.PhotonVisionTargetRawData;
import org.littletonrobotics.junction.Logger;
import java.util.ArrayList;
import java.util.List;

public class PhotonVisionGameObjectFilters extends PhotonVisionFiltered {

	public PhotonVisionGameObjectFilters(CameraConfiguration[] cameraConfigurations, String logPath) {
		super(cameraConfigurations, logPath);
	}

	public GameObjectPoseObservation getObservationFromRawData(PhotonVisionTargetRawData gameObjectData) {
		return new GameObjectPoseObservation(gameObjectData.targetPose(), gameObjectData.timestamp());
	}

	public List<GameObjectPoseObservation> getAllGameObjectPoses() {
		List<GameObjectPoseObservation> output = new ArrayList<>();
		for (PhotonVisionTargetRawData gameObjectData : getAllFilteredData()) {
			output.add(getObservationFromRawData(gameObjectData));
		}
		return output;
	}

	@Override
	protected boolean keepPhotonVisionData(PhotonVisionTargetRawData targetData) {
		return !isDataLatencyTooHigh(targetData) && !isDataTooAmbiguous(targetData);
	}

	public void logAllGameObjectPoses() {
		for (GameObjectPoseObservation observation : getAllGameObjectPoses()) {
			Logger.recordOutput(String.valueOf(observation.timestamp()), observation.pose());
		}
	}

	@Override
	protected void subsystemPeriodic() {
		logAllGameObjectPoses();
	}

}
