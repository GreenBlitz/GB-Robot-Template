package frc.robot.vision.photonvision.photonvisionfilters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.poseestimator.observations.VisionObservation;
import frc.robot.vision.VisionRawData;
import frc.robot.vision.aprilTags.AprilTagFilters;
import frc.robot.vision.aprilTags.AprilTagFiltersTolerances;
import frc.robot.vision.photonvision.CameraConfiguration;
import frc.robot.vision.photonvision.PhotonVisionConstants;

import java.util.ArrayList;
import java.util.function.Function;

public class PhotonVisionAprilTagFilters extends PhotonVisionFiltered {

	public PhotonVisionAprilTagFilters(
		CameraConfiguration[] cameraConfigurations,
		String logPath,
		Function<Double, Pose2d> getEstimatedPoseAtTimestamp,
		AprilTagFiltersTolerances tolerances
	) {
		super(cameraConfigurations, logPath, getEstimatedPoseAtTimestamp, tolerances);
	}

	@Override
	protected boolean
		keepPhotonVisionData(VisionRawData targetData, Pose2d currentPose, double aprilTagHeightMeters, AprilTagFiltersTolerances tolerances) {
		return AprilTagFilters.keepLimelightData(targetData, currentPose, aprilTagHeightMeters, tolerances);
	}

	private VisionObservation getObservationFromPhotonPose(VisionRawData poseData) {
		Pose3d robotPose3d = poseData.targetPose();
		Pose2d robotPose2d = new Pose2d(robotPose3d.getX(), robotPose3d.getY(), new Rotation2d(robotPose3d.getRotation().getZ()));
		return new VisionObservation(robotPose2d, getStandardDeviations(poseData), poseData.timestamp());
	}

	public ArrayList<VisionObservation> getAllFilteredVisionObservations() {
		ArrayList<VisionObservation> output = new ArrayList<>();
		for (VisionRawData poseData : getAllFilteredData()) {
			output.add(getObservationFromPhotonPose(poseData));
		}
		return output;
	}

	private static double[] getStandardDeviations(VisionRawData targetData) {
		double ambiguity = targetData.ambiguity();
		double positionStandardDeviation = ambiguity / PhotonVisionConstants.AMBIGUITY_TO_LOCATION_STANDARD_DEVIATIONS_FACTOR;
		double rotationStandardDeviation = ambiguity / PhotonVisionConstants.AMBIGUITY_TO_ROTATION_STANDARD_DEVIATIONS_FACTOR;
		return new double[] {positionStandardDeviation, positionStandardDeviation, rotationStandardDeviation};
	}

}
