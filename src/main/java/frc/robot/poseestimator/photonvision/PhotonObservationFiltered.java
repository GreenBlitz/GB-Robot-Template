package frc.robot.poseestimator.photonvision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.poseestimator.observations.VisionObservation;
import frc.robot.poseestimator.photonvision.photoncamera.GBPhotonCamera;
import frc.robot.poseestimator.photonvision.photoncamera.PhotonTargetData;

import java.util.ArrayList;
import java.util.Optional;

public class PhotonObservationFiltered {

	private final ArrayList<GBPhotonCamera> cameras;

	public PhotonObservationFiltered(CameraConfiguration[] cameraConfigurations) {
		cameras = new ArrayList<>();
		for (CameraConfiguration cameraConfiguration : cameraConfigurations) {
			cameras.add(new GBPhotonCamera(cameraConfiguration));
		}
	}

	private boolean isDataTooAmbiguous(PhotonTargetData targetData) {
		return targetData.ambiguity() >= PhotonConstants.MAXIMUM_ALLOWED_AMBIGUITY;
	}

	private boolean isDataLatencyTooHigh(PhotonTargetData targetData) {
		return targetData.latency() >= PhotonConstants.MAXIMUM_ALLOWED_LATENCY;
	}

	private boolean keepLimelightData(PhotonTargetData targetData) {
		return !isDataTooAmbiguous(targetData) && !isDataLatencyTooHigh(targetData);
	}

	private ArrayList<PhotonTargetData> getAllTargetData() {
		ArrayList<PhotonTargetData> output = new ArrayList<>();
		for (GBPhotonCamera camera : cameras) {
			Optional<PhotonTargetData> bestTarget = camera.getBestTargetData();
			bestTarget.ifPresent(output::add);
		}
		return output;
	}

	public ArrayList<VisionObservation> getAllFilteredObservations() {
		ArrayList<VisionObservation> output = new ArrayList<>();
		for (PhotonTargetData targetData : getAllTargetData()) {
			if (keepLimelightData(targetData)) {
				output.add(calculateObservationFromTargetData(targetData));
			}
		}
		return output;
	}

	private VisionObservation calculateObservationFromTargetData(PhotonTargetData targetData) {
		Pose3d robot3DPose = targetData.robotPose();
		Pose2d robot2DPose = new Pose2d(
			robot3DPose.getX(),
			robot3DPose.getY(),
			Rotation2d.fromRadians(robot3DPose.getRotation().getZ())
		);
		return new VisionObservation(robot2DPose, getStandardDeviations(targetData), targetData.timestamp());
	}

	public double[] getStandardDeviations(PhotonTargetData targetData) {
		double ambiguity = targetData.ambiguity();
		return new double[] {
			ambiguity / PhotonConstants.AMBIGUITY_TO_LOCATION_STANDARD_DEVIATIONS_FACTOR,
			ambiguity / PhotonConstants.AMBIGUITY_TO_LOCATION_STANDARD_DEVIATIONS_FACTOR,
			ambiguity / PhotonConstants.AMBIGUITY_TO_ROTATION_STANDARD_DEVIATIONS_FACTOR};
	}

}
