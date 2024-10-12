package frc.robot.vision.photonvision.photonvisionfilters;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.poseestimator.observations.GameObjectPoseObservation;
import frc.robot.vision.VisionRawData;
import frc.robot.vision.aprilTags.AprilTagFiltersTolerances;
import frc.robot.vision.photonvision.CameraConfiguration;
import org.littletonrobotics.junction.Logger;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

public class PhotonVisionGameObjectFilters extends PhotonVisionFiltered {

	public PhotonVisionGameObjectFilters(
		CameraConfiguration[] cameraConfigurations,
		String logPath,
		Function<Double, Pose2d> getEstimatedPoseAtTimestamp,
		AprilTagFiltersTolerances tolerances
	) {
		super(cameraConfigurations, logPath, getEstimatedPoseAtTimestamp, tolerances);
	}

	@Override
	protected boolean
		keepPhotonVisionData(VisionRawData targetData, Pose2d currentPose, double aprilTagHeightMeters, AprilTagFiltersTolerances tolerances) {
		return false;
	}

	public GameObjectPoseObservation getObservationFromRawData(VisionRawData gameObjectData) {
		return new GameObjectPoseObservation(gameObjectData.targetPose(), gameObjectData.timestamp());
	}

	public List<GameObjectPoseObservation> getAllGameObjectPoses() {
		List<GameObjectPoseObservation> output = new ArrayList<>();
		for (VisionRawData gameObjectData : getAllFilteredData()) {
			output.add(getObservationFromRawData(gameObjectData));
		}
		return output;
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
