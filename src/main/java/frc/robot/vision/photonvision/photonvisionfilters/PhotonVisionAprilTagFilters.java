package frc.robot.vision.photonvision.photonvisionfilters;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.poseestimator.PoseEstimationMath;
import frc.robot.poseestimator.observations.VisionObservation;
import frc.robot.vision.VisionRawData;
import frc.robot.vision.aprilTags.AprilTagFilters;
import frc.robot.vision.aprilTags.AprilTagFiltersTolerances;
import frc.robot.vision.photonvision.CameraConfiguration;
import frc.utils.Conversions;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.function.Function;

public class PhotonVisionAprilTagFilters extends PhotonVisionFiltered {

	private final Function<Double, Pose2d> getEstimatedPoseAtTimestamp;
	private final AprilTagFiltersTolerances tolerances;

	public PhotonVisionAprilTagFilters(
		CameraConfiguration[] cameraConfigurations,
		String logPath,
		Function<Double, Pose2d> getEstimatedPoseAtTimestamp,
		AprilTagFiltersTolerances tolerances
	) {
		super(cameraConfigurations, logPath);
		this.getEstimatedPoseAtTimestamp = getEstimatedPoseAtTimestamp;
		this.tolerances = tolerances;
	}

	@Override
	protected boolean keepPhotonVisionData(VisionRawData targetData) {
		return AprilTagFilters.keepLimelightData(targetData, calculateCurrentPoseEstimation(), tolerances);
	}

	public ArrayList<VisionObservation> getAllFilteredVisionObservations() {
		ArrayList<VisionObservation> output = new ArrayList<>();
		for (VisionRawData poseData : getAllFilteredData()) {
			output.add(PoseEstimationMath.rawDataToObservation(poseData, calculateCurrentPoseEstimation()));
		}
		return output;
	}

	private Pose2d calculateCurrentPoseEstimation() {
		return getEstimatedPoseAtTimestamp.apply(Conversions.microSecondsToSeconds(Logger.getRealTimestamp()));
	}

}
