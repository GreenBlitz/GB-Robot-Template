package frc.robot.vision.photonvision.photonvisionfilters;

import frc.robot.subsystems.GBSubsystem;
import frc.robot.vision.photonvision.*;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.Optional;

public abstract class PhotonVisionFiltered extends GBSubsystem {

	private final ArrayList<PhotonVisionCamera> cameras;

	public PhotonVisionFiltered(CameraConfiguration[] cameraConfigurations, String logPath) {
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

	public ArrayList<PhotonVisionTargetRawData> getAllFilteredData() {
		ArrayList<PhotonVisionTargetRawData> output = new ArrayList<>();
		for (PhotonVisionTargetRawData targetData : getAllTargetData()) {
			if (keepPhotonVisionData(targetData)) {
				output.add(targetData);
			}
		}

		return output;
	}

	protected ArrayList<PhotonVisionTargetRawData> getAllTargetData() {
		ArrayList<PhotonVisionTargetRawData> output = new ArrayList<>();
		for (PhotonVisionCamera camera : cameras) {
			Optional<ArrayList<Optional<PhotonVisionTargetRawData>>> targets = camera.getTargetsData();
			if (targets.isPresent()) {
				for (Optional<PhotonVisionTargetRawData> targetRawData : targets.get()) {
					targetRawData.ifPresent(output::add);
				}
			}
		}

		return output;
	}

	private void logAllFilteredData() {
		for (PhotonVisionTargetRawData targetData : getAllFilteredData()) {
			Logger.recordOutput(
				super.getLogPath() + targetData.cameraName() + "Time" + targetData.timestamp(),
				targetData.targetPose()
			);
		}
	}

	private void logAllTargetData() {
		for (PhotonVisionTargetRawData targetData : getAllTargetData()) {
			Logger.recordOutput(
				super.getLogPath() + targetData.cameraName() + "Time" + targetData.timestamp(),
				targetData.targetPose()
			);
		}
	}

	@Override
	protected void subsystemPeriodic() {
		logAllTargetData();
	}

}
