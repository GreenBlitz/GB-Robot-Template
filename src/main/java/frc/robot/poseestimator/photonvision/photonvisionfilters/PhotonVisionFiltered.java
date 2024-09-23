package frc.robot.poseestimator.photonvision.photonvisionfilters;

import frc.robot.poseestimator.photonvision.*;
import frc.utils.GBSubsystem;
import java.util.ArrayList;
import java.util.Optional;

public abstract class PhotonVisionFiltered extends GBSubsystem {

	private final ArrayList<PhotonVisionCamera> cameras;

	public PhotonVisionFiltered(CameraConfiguration[] cameraConfigurations, PhotonVisionTarget target, String logPath) {
		super(logPath);
		this.cameras = new ArrayList<>();
		for (CameraConfiguration cameraConfiguration : cameraConfigurations) {
			this.cameras.add(new PhotonVisionCamera(cameraConfiguration));
		}
	}

	protected boolean isDataTooAmbiguous(PhotonTargetRawData targetData) {
		return targetData.ambiguity() >= PhotonVisionConstants.MAXIMUM_ALLOWED_AMBIGUITY;
	}

	protected boolean isDataLatencyTooHigh(PhotonTargetRawData targetData) {
		return targetData.latency() >= PhotonVisionConstants.MAXIMUM_ALLOWED_LATENCY;
	}

	protected abstract boolean keepPhotonVisionData(PhotonTargetRawData targetData);

	protected ArrayList<PhotonTargetRawData> getAllTargetData() {
		ArrayList<PhotonTargetRawData> output = new ArrayList<>();
		for (PhotonVisionCamera camera : cameras) {
			Optional<PhotonTargetRawData> bestTarget = camera.getBestTargetData();
			bestTarget.ifPresent(output::add);
		}
		return output;
	}

	public double[] getStandardDeviations(PhotonTargetRawData targetData) {
		double ambiguity = targetData.ambiguity();
		return new double[] {
			ambiguity / PhotonVisionConstants.AMBIGUITY_TO_LOCATION_STANDARD_DEVIATIONS_FACTOR,
			ambiguity / PhotonVisionConstants.AMBIGUITY_TO_LOCATION_STANDARD_DEVIATIONS_FACTOR,
			ambiguity / PhotonVisionConstants.AMBIGUITY_TO_ROTATION_STANDARD_DEVIATIONS_FACTOR};
	}

}
