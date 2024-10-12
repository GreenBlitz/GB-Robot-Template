package frc.robot.vision.photonvision.photonvisionfilters;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.vision.VisionRawData;
import frc.robot.vision.aprilTags.AprilTagFiltersTolerances;
import frc.robot.vision.photonvision.*;
import frc.utils.Conversions;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.Optional;
import java.util.function.Function;

public abstract class PhotonVisionFiltered extends GBSubsystem {

	private final ArrayList<PhotonVisionCamera> cameras;
	private final Function<Double, Pose2d> getEstimatedPoseAtTimestamp;
	private final AprilTagFiltersTolerances tolerances;

	public PhotonVisionFiltered(
		CameraConfiguration[] cameraConfigurations,
		String logPath,
		Function<Double, Pose2d> getEstimatedPoseAtTimestamp,
		AprilTagFiltersTolerances tolerances
	) {
		super(logPath);
		this.cameras = new ArrayList<>();
		for (CameraConfiguration cameraConfiguration : cameraConfigurations) {
			this.cameras.add(new PhotonVisionCamera(cameraConfiguration));
		}
		this.getEstimatedPoseAtTimestamp = getEstimatedPoseAtTimestamp;
		this.tolerances = tolerances;
	}

	protected abstract boolean
		keepPhotonVisionData(VisionRawData targetData, Pose2d currentPose, double aprilTagHeightMeters, AprilTagFiltersTolerances tolerances);

	public ArrayList<VisionRawData> getAllFilteredData() {
		ArrayList<VisionRawData> output = new ArrayList<>();
		for (VisionRawData targetData : getAllTargetData()) {
			if (
				keepPhotonVisionData(
					targetData,
					getEstimatedPoseAtTimestamp.apply(Conversions.microSecondsToSeconds(Logger.getRealTimestamp())),
					PhotonVisionConstants.APRIL_TAG_FIELD_LAYOUT.getTagPose(0).get().getZ(),
					tolerances
				)
			) {
				output.add(targetData);
			}
		}

		return output;
	}

	protected ArrayList<VisionRawData> getAllTargetData() {
		ArrayList<VisionRawData> output = new ArrayList<>();
		for (PhotonVisionCamera camera : cameras) {
			Optional<ArrayList<Optional<VisionRawData>>> targets = camera.getTargetsData();
			if (targets.isPresent()) {
				for (Optional<VisionRawData> targetRawData : targets.get()) {
					targetRawData.ifPresent(output::add);
				}
			}
		}

		return output;
	}

	private void logAllFilteredData() {
		for (VisionRawData targetData : getAllFilteredData()) {
			Logger.recordOutput(super.getLogPath() + targetData.cameraName() + "Time" + targetData.timestamp(), targetData.targetPose());
		}
	}

	private void logAllTargetData() {
		for (VisionRawData targetData : getAllTargetData()) {
			Logger.recordOutput(super.getLogPath() + targetData.cameraName() + "Time" + targetData.timestamp(), targetData.targetPose());
		}
	}

	@Override
	protected void subsystemPeriodic() {
		logAllTargetData();
	}

}
