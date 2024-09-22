package frc.robot.poseestimator.photonvision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.poseestimator.observations.VisionObservation;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.Optional;

public class PhotonObservationFiltered {

	private final ArrayList<PhotonVisionCamera> cameras;

	public PhotonObservationFiltered(CameraConfiguration[] cameraConfigurations) {
		this.cameras = new ArrayList<>();
		for (CameraConfiguration cameraConfiguration : cameraConfigurations) {
			this.cameras.add(new PhotonVisionCamera(cameraConfiguration));
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
			if (bestTarget.isPresent() && bestTarget.get().target() == PhotonVisionTarget.APRIL_TAG) {
				output.add(bestTarget.get());
			}
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
		Pose3d robotPose3d = poseData.robotPose();
		Pose2d robotPose2d = new Pose2d(robotPose3d.getX(), robotPose3d.getY(), new Rotation2d(robotPose3d.getRotation().getZ()));
		return new VisionObservation(robotPose2d, getStandardDeviations(poseData), poseData.timestamp());
	}

	public double[] getStandardDeviations(PhotonPoseData targetData) {
		double ambiguity = targetData.ambiguity();
		return new double[] {
			ambiguity / PhotonVisionConstants.AMBIGUITY_TO_LOCATION_STANDARD_DEVIATIONS_FACTOR,
			ambiguity / PhotonVisionConstants.AMBIGUITY_TO_LOCATION_STANDARD_DEVIATIONS_FACTOR,
			ambiguity / PhotonVisionConstants.AMBIGUITY_TO_ROTATION_STANDARD_DEVIATIONS_FACTOR};
	}
	
	public void logAll() {
		for (VisionObservation observation : getAllFilteredObservations()) {
			Logger.recordOutput(
					String.valueOf(observation.timestamp()),
					observation.visionPose()
			);
		}
	}

}
