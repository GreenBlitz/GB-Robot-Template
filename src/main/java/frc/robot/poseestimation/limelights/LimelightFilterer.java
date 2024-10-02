package frc.robot.poseestimation.limelights;

import frc.robot.poseestimation.poseestimator.IPoseEstimator;
import frc.robot.poseestimation.observations.VisionObservation;
import frc.utils.GBSubsystem;
import org.littletonrobotics.junction.Logger;
import java.util.ArrayList;
import java.util.List;

public class LimelightFilterer extends GBSubsystem {

	private final MultiLimelightsRawData multiLimelightsRawData;
	private final LimelightFiltererConfig config;
	private final IPoseEstimator poseEstimator;

	public LimelightFilterer(LimelightFiltererConfig config, IPoseEstimator poseEstimator) {
		super(config.logPath());

		this.multiLimelightsRawData = new MultiLimelightsRawData(config.limelightsNames(), config.hardwareLogPath());
		this.config = config;
		this.poseEstimator = poseEstimator;
	}

	public void updateGyroAngles(double[] gyroAnglesValues) {
		multiLimelightsRawData.updateGyroAngles(gyroAnglesValues);
	}

	public List<VisionObservation> getFilteredVisionObservations() {
		ArrayList<VisionObservation> estimates = new ArrayList<>();

		for (LimelightRawData limelightRawData : multiLimelightsRawData.getAllAvailableLimelightData()) {
			if (keepLimelightData(limelightRawData)) {
				estimates.add(rawDataToObservation(limelightRawData));
			}
		}

		return estimates;
	}

	public List<VisionObservation> getAllAvailableLimelightData() {
		ArrayList<VisionObservation> estimates = new ArrayList<>();

		for (LimelightRawData limelightRawData : multiLimelightsRawData.getAllAvailableLimelightData()) {
			estimates.add(rawDataToObservation(limelightRawData));
		}

		return estimates;
	}

	private VisionObservation rawDataToObservation(LimelightRawData limelightRawData) {
		double standardTransformDeviation = getDynamicStandardTransformDeviations(limelightRawData);
		double[] standardDeviations = new double[] {
			standardTransformDeviation,
			standardTransformDeviation,
			LimeLightConstants.STANDARD_DEVIATION_VISION_ANGLE};

		return new VisionObservation(
			limelightRawData.estimatedPose().toPose2d(),
			standardDeviations,
			limelightRawData.timestamp()
		);
	}

	private boolean keepLimelightData(LimelightRawData limelightRawData) {
		return LimelightFilters.isAprilTagInProperHeight(limelightRawData)
			&& LimelightFilters.isLimelightOutputInTolerance(limelightRawData, poseEstimator.getEstimatedPose())
			&& LimelightFilters.isRollZero(limelightRawData)
			&& LimelightFilters.isPitchZero(limelightRawData)
			&& LimelightFilters.isRobotOnGround(limelightRawData);
	}

	public void logEstimatedPositions() {
		List<VisionObservation> observations = getFilteredVisionObservations();

		for (int i = 0; i < observations.size(); i++) {
			Logger.recordOutput(
				super.getLogPath() + LimeLightConstants.ESTIMATION_LOGPATH_PREFIX + i + "Time" + observations.get(i).timestamp(),
				observations.get(i).visionPose()
			);
		}
	}

	private double getDynamicStandardTransformDeviations(LimelightRawData limelightRawData) {
		return limelightRawData.distanceFromAprilTag() / LimeLightConstants.APRIL_TAG_DISTANCE_TO_STANDARD_DEVIATIONS_FACTOR;
	}

	@Override
	protected void subsystemPeriodic() {}

}
