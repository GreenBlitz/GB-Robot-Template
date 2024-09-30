package frc.robot.poseestimator.limelights;

import frc.robot.poseestimator.GBPoseEstimator;
import frc.robot.poseestimator.observations.VisionObservation;
import frc.utils.GBSubsystem;
import org.littletonrobotics.junction.Logger;
import java.util.ArrayList;
import java.util.List;

public class VisionObservationFiltered extends GBSubsystem {

	private final MultiLimelightsRawData limelightHardware;
	private final VisionObservationFilteredConfig config;
	private final GBPoseEstimator poseEstimator;

	public VisionObservationFiltered(VisionObservationFilteredConfig config, GBPoseEstimator poseEstimator) {
		super(config.logPath());

		this.limelightHardware = new MultiLimelightsRawData(config.limelightsNames(), config.hardwareLogPath());
		this.config = config;
		this.poseEstimator = poseEstimator;
	}

	public List<VisionObservation> getFilteredVisionObservations() {
		ArrayList<VisionObservation> estimates = new ArrayList<>();

		for (LimelightRawData limelightRawData : limelightHardware.getAllAvailableLimelightData()) {
			if (keepLimelightData(limelightRawData)) {
				estimates.add(rawDataToObservation(limelightRawData));
			}
		}

		return estimates;
	}

	private VisionObservation rawDataToObservation(LimelightRawData limelightRawData) {
		double standardTransformDeviation = getDynamicStandardTransformDeviations(limelightRawData);
		double[] standardDeviations = new double[] {
			standardTransformDeviation,
			standardTransformDeviation,
			VisionConstants.STANDARD_DEVIATION_VISION_ANGLE};

		return new VisionObservation(limelightRawData.estimatedPose().toPose2d(), standardDeviations, limelightRawData.timestamp());
	}

	private boolean keepLimelightData(LimelightRawData limelightRawData) {
		return LimelightFilters.isAprilTagInProperHeight(limelightRawData)
			&& LimelightFilters.isLimelightOutputInTolerance(limelightRawData)
			&& LimelightFilters.isRollZero(limelightRawData)
			&& LimelightFilters.isPitchZero(limelightRawData)
			&& LimelightFilters.isRobotOnGround(limelightRawData);
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

	private double getDynamicStandardTransformDeviations(LimelightRawData limelightRawData) {
		return 1;
	}

	@Override
	protected void subsystemPeriodic() {}

}
