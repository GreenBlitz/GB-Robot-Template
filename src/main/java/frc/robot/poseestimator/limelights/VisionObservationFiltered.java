package frc.robot.poseestimator.limelights;

import frc.robot.constants.Field;
import frc.robot.poseestimator.observations.VisionObservation;
import frc.utils.GBSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;

public class VisionObservationFiltered extends GBSubsystem {

	private MultiLimelightsRawData limelightHardware;
	private VisionObservationFilteredConfig config;

	public VisionObservationFiltered(VisionObservationFilteredConfig config) {
		super(config.logPath());

		this.limelightHardware = new MultiLimelightsRawData(config.limelightsNames(), config.hardwareLogPath());
		this.config = config;
	}

	public List<VisionObservation> getFilteredVisionObservations() {
		ArrayList<VisionObservation> estimates = new ArrayList<>();

		for (LimelightRawData limelightRawData : limelightHardware.getAllAvailableLimelightData()) {
			Logger.recordOutput(super.getLogPath() + limelightRawData.timeStamp(), limelightRawData.AprilTagHeight());
			if (!filterOutLimelightData(limelightRawData)) {
				double standardDeviation = getDynamicStandardDeviations(limelightRawData);
				double[] standardDeviations = new double[] {standardDeviation};

				estimates.add(
					new VisionObservation(limelightRawData.EstimatedPosition(), standardDeviations, limelightRawData.timeStamp())
				);
			}
		}

		return estimates;
	}

	private boolean isLimelightedOutputInTolerance(LimelightRawData limelightRawData) {
		// ! THIS SHOULDN'T BE COMMENTED OUT
		// ! this is a placeholder since this filter is depended on the poseestimatorx
		return true;
//		Pose2d currentPoseObservation = NetworkTables...;

//		Pose2d limelightPosition = limelightData.EstimatedPosition();
//		Transform2d transformDifference = limelightPosition.minus(currentPoseObservation);
//		Rotation2d rotationDifference = limelightPosition.getRotation().minus(currentPoseObservation.getRotation());
//
//		return transformDifference.getTranslation().getNorm() <= config.positionNormTolerance()
//			&& rotationDifference.getDegrees() <= config.rotationTolerance().getDegrees();
	}

	private boolean isAprilTagInProperHeight(LimelightRawData limelightRawData) {
		boolean aprilTagHeightConfidence = Math.abs(limelightRawData.AprilTagHeight() - Field.APRIL_TAG_HEIGHT_METERS)
			< VisionConstants.APRIL_TAG_HEIGHT_TOLERANCE_METERS;
		return aprilTagHeightConfidence;
	}

	private boolean filterOutLimelightData(LimelightRawData limelightRawData) {
		return !(isAprilTagInProperHeight(limelightRawData) && isLimelightedOutputInTolerance(limelightRawData));
	}

	public void logEstimatedPositions() {
		List<VisionObservation> observations = getFilteredVisionObservations();

		for (int i = 0; i < observations.size(); i++) {
			Logger.recordOutput(
				super.getLogPath() + VisionConstants.ESTIMATION_LOGPATH_PREFIX + i + observations.get(i).timestamp(),
				observations.get(i).visionPose()
			);
		}
	}

	private double getDynamicStandardDeviations(LimelightRawData limelightRawData) {
		return limelightRawData.DistanceFromAprilTag() / VisionConstants.APRIL_TAG_DiSTANCE_TO_STANDARD_DEVIATIONS_FACTOR;
	}

	public VisionObservation getFirstAvailableTarget() {
		return getFilteredVisionObservations().get(0);
	}

	@Override
	protected void subsystemPeriodic() {}

}
