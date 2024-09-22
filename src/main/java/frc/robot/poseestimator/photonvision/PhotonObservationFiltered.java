package frc.robot.poseestimator.photonvision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.poseestimator.observations.VisionObservation;

import java.util.ArrayList;
import java.util.Optional;

public class PhotonObservationFiltered {

	private final ArrayList<PhotonVisionCamera> cameras;

	public PhotonObservationFiltered(CameraConfiguration[] cameraConfigurations) {
		cameras = new ArrayList<>();
		for (CameraConfiguration cameraConfiguration : cameraConfigurations) {
			cameras.add(new PhotonVisionCamera(cameraConfiguration));
		}
	}

	private boolean isDataTooAmbiguous(PhotonPoseData targetData) {
		return targetData.ambiguity() >= PhotonVisionConstants.MAXIMUM_ALLOWED_AMBIGUITY;
	}

	private boolean isDataLatencyTooHigh(PhotonPoseData targetData) {
		return targetData.latency() >= PhotonVisionConstants.MAXIMUM_ALLOWED_LATENCY;
	}

	private boolean keepPhotonVisionData(PhotonPoseData targetData) {
		return !isDataTooAmbiguous(targetData) && !isDataLatencyTooHigh(targetData);
	}

	private ArrayList<PhotonPoseData> getAllPoseData() {
		ArrayList<PhotonPoseData> output = new ArrayList<>();
		for (PhotonVisionCamera camera : cameras) {
			Optional<PhotonPoseData> bestTarget = camera.getBestTargetData();
			bestTarget.ifPresent(bestTargetData -> {
				if (bestTargetData.target() == PhotonVisionTarget.APRIL_TAG) {
					output.add(bestTargetData);
				}
			});
		}
		return output;
	}

	public ArrayList<VisionObservation> getAllFilteredObservations() {
		ArrayList<VisionObservation> output = new ArrayList<>();
		for (PhotonPoseData poseData : getAllPoseData()) {
			if (keepPhotonVisionData(poseData)) {
				output.add(calculateObservationFromPoseData(poseData));
			}
		}
		return output;
	}

	private VisionObservation calculateObservationFromPoseData(PhotonPoseData poseData) {
		Pose3d robot3DPose = poseData.robotPose();
		Pose2d robot2DPose = new Pose2d(
			robot3DPose.getX(),
			robot3DPose.getY(),
			Rotation2d.fromRadians(robot3DPose.getRotation().getZ())
		);
		return new VisionObservation(robot2DPose, getStandardDeviations(poseData), poseData.timestamp());
	}

	public double[] getStandardDeviations(PhotonPoseData targetData) {
		double ambiguity = targetData.ambiguity();
		return new double[] {
			ambiguity / PhotonVisionConstants.AMBIGUITY_TO_LOCATION_STANDARD_DEVIATIONS_FACTOR,
			ambiguity / PhotonVisionConstants.AMBIGUITY_TO_LOCATION_STANDARD_DEVIATIONS_FACTOR,
			ambiguity / PhotonVisionConstants.AMBIGUITY_TO_ROTATION_STANDARD_DEVIATIONS_FACTOR};
	}

}
