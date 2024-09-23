package frc.robot.poseestimator.photonvision.photonvisionfilters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.poseestimator.observations.VisionObservation;
import frc.robot.poseestimator.photonvision.CameraConfiguration;
import frc.robot.poseestimator.photonvision.PhotonTargetRawData;
import frc.robot.poseestimator.photonvision.PhotonVisionConstants;
import frc.robot.poseestimator.photonvision.PhotonVisionTarget;

import java.util.ArrayList;

public class PhotonVisionAprilTagFilters extends PhotonVisionFiltered {

	public PhotonVisionAprilTagFilters(CameraConfiguration[] cameraConfigurations, PhotonVisionTarget target, String logPath) {
		super(cameraConfigurations, target, logPath);
	}

	@Override
	protected boolean keepPhotonVisionData(PhotonTargetRawData targetData) {
		return isAprilTagWithinRange(targetData) && !isDataTooAmbiguous(targetData) && !isDataLatencyTooHigh(targetData);
	}

	private VisionObservation getObservationFromPhotonPose(PhotonTargetRawData poseData) {
		Pose3d robotPose3d = poseData.targetPose();
		Pose2d robotPose2d = new Pose2d(robotPose3d.getX(), robotPose3d.getY(), new Rotation2d(robotPose3d.getRotation().getZ()));
		return new VisionObservation(robotPose2d, getStandardDeviations(poseData), poseData.timestamp());
	}

	public ArrayList<VisionObservation> getAllFilteredVisionObservations() {
		ArrayList<VisionObservation> output = new ArrayList<>();
		for (PhotonTargetRawData poseData : getAllTargetData()) {
			if (keepPhotonVisionData(poseData)) {
				output.add(getObservationFromPhotonPose(poseData));
			}
		}
		return output;
	}

	private boolean isAprilTagWithinRange(PhotonTargetRawData targetData) {
		double height = targetData.targetPose().getZ();
		return PhotonVisionConstants.APRIL_TAG_MINIMUM_HEIGHT <= height
			&& PhotonVisionConstants.APRIL_TAG_MINIMUM_HEIGHT >= height;
	}

	@Override
	protected void subsystemPeriodic() {}

}
