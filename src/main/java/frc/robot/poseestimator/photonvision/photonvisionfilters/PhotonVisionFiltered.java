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

	protected boolean isDataTooAmbiguous(PhotonVisionTargetRawData targetData) {
		return targetData.ambiguity() >= PhotonVisionConstants.MAXIMUM_ALLOWED_AMBIGUITY;
	}

	protected boolean isDataLatencyTooHigh(PhotonVisionTargetRawData targetData) {
		return targetData.latency() >= PhotonVisionConstants.MAXIMUM_ALLOWED_LATENCY;
	}

	protected abstract boolean keepPhotonVisionData(PhotonVisionTargetRawData targetData);

	protected ArrayList<PhotonVisionTargetRawData> getAllTargetData() {
		ArrayList<PhotonVisionTargetRawData> output = new ArrayList<>();
		for (PhotonVisionCamera camera : cameras) {
			Optional<PhotonVisionTargetRawData> bestTarget = camera.getBestTargetData();
			bestTarget.ifPresent(output::add);
		}
		return output;
	}

}
