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
			Logger.recordOutput(super.getLogPath() + limelightRawData.timeStamp(), limelightRawData.aprilTagHeight());
			if (keepLimelightData(limelightRawData)) {
				double standardDeviation = getDynamicStandardDeviations(limelightRawData);
				double[] standardDeviations = new double[] {standardDeviation, standardDeviation, standardDeviation};

				estimates.add(
					new VisionObservation(limelightRawData.estimatedPosition(), standardDeviations, limelightRawData.timeStamp())
				);
			}
		}

		return estimates;
	}

	private boolean isLimelightOutputInTolerance(LimelightRawData limelightRawData) {
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
		boolean aprilTagHeightConfidence = Math.abs(limelightRawData.aprilTagHeight() - Field.APRIL_TAG_HEIGHT_METERS)
			<= VisionConstants.APRIL_TAG_HEIGHT_TOLERANCE_METERS;
		return aprilTagHeightConfidence;
	}

	private boolean keepLimelightData(LimelightRawData limelightRawData) {
		return isAprilTagInProperHeight(limelightRawData) && isLimelightOutputInTolerance(limelightRawData);
	}

	public void logEstimatedPositions() {
		List<VisionObservation> observations = getFilteredVisionObservations();

		for (int i = 0; i < observations.size(); i++) {
			Logger.recordOutput(
				super.getLogPath() + VisionConstants.ESTIMATION_LOGPATH_PREFIX + i + "Time" + observations.get(i).timestamp(),
				observations.get(i).visionPose()
			);
		}
	}

	private double getDynamicStandardDeviations(LimelightRawData limelightRawData) {
		return limelightRawData.distanceFromAprilTag() / VisionConstants.APRIL_TAG_DISTANCE_TO_STANDARD_DEVIATIONS_FACTOR;
	}

	@Override
	protected void subsystemPeriodic() {}

}
