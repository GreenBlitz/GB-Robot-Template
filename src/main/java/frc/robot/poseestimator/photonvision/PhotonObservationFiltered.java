package frc.robot.poseestimator.photonvision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.poseestimator.observations.VisionObservation;
import frc.utils.GBSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.Optional;

public class PhotonObservationFiltered extends GBSubsystem {

	private final ArrayList<PhotonVisionCamera> cameras;
	private final PhotonVisionTarget target;

	public PhotonObservationFiltered(CameraConfiguration[] cameraConfigurations, PhotonVisionTarget target, String logPath) {
		super(logPath);
		this.cameras = new ArrayList<>();
		this.target = target;
		for (CameraConfiguration cameraConfiguration : cameraConfigurations) {
			this.cameras.add(new PhotonVisionCamera(cameraConfiguration));
		}
	}

	private boolean isDataTooAmbiguous(PhotonTargetRawData targetData) {
		return targetData.ambiguity() >= PhotonVisionConstants.MAXIMUM_ALLOWED_AMBIGUITY;
	}

	private boolean isDataLatencyTooHigh(PhotonTargetRawData targetData) {
		return targetData.latency() >= PhotonVisionConstants.MAXIMUM_ALLOWED_LATENCY;
	}

	private boolean isAprilTagWithinRange(PhotonTargetRawData targetData) {
		double height = targetData.robotPose().getZ();
		return PhotonVisionConstants.APRIL_TAG_MINIMUM_HEIGHT <= height
			&& PhotonVisionConstants.APRIL_TAG_MINIMUM_HEIGHT >= height;
	}

	private boolean keepPhotonVisionData(PhotonTargetRawData targetData) {
		if (target == PhotonVisionTarget.APRIL_TAG && !isAprilTagWithinRange(targetData)) {
			return false;
		}
		return !isDataTooAmbiguous(targetData) && !isDataLatencyTooHigh(targetData);
	}

	private ArrayList<PhotonTargetRawData> getAllTargetData() {
		ArrayList<PhotonTargetRawData> output = new ArrayList<>();
		for (PhotonVisionCamera camera : cameras) {
			Optional<PhotonTargetRawData> bestTarget = camera.getBestTargetData();
			if (bestTarget.isPresent() && bestTarget.get().target() == target) {
				output.add(bestTarget.get());
			}
		}
		return output;
	}

	public ArrayList<VisionObservation> getAllFilteredTargetObservations() {
		ArrayList<VisionObservation> output = new ArrayList<>();
		for (PhotonTargetRawData poseData : getAllTargetData()) {
			if (keepPhotonVisionData(poseData)) {
				output.add(getObservationFromPhotonPose(poseData));
			}
		}
		return output;
	}

	private VisionObservation getObservationFromPhotonPose(PhotonTargetRawData poseData) {
		Pose3d robotPose3d = poseData.robotPose();
		Pose2d robotPose2d = new Pose2d(robotPose3d.getX(), robotPose3d.getY(), new Rotation2d(robotPose3d.getRotation().getZ()));
		return new VisionObservation(robotPose2d, getStandardDeviations(poseData), poseData.timestamp());
	}

	public double[] getStandardDeviations(PhotonTargetRawData targetData) {
		double ambiguity = targetData.ambiguity();
		return new double[] {
			ambiguity / PhotonVisionConstants.AMBIGUITY_TO_LOCATION_STANDARD_DEVIATIONS_FACTOR,
			ambiguity / PhotonVisionConstants.AMBIGUITY_TO_LOCATION_STANDARD_DEVIATIONS_FACTOR,
			ambiguity / PhotonVisionConstants.AMBIGUITY_TO_ROTATION_STANDARD_DEVIATIONS_FACTOR};
	}

	public void logAll() {
		for (VisionObservation observation : getAllFilteredTargetObservations()) {
			Logger.recordOutput(super.getLogPath() + observation.timestamp(), observation.visionPose());
		}
	}

	@Override
	protected void subsystemPeriodic() {}

}
