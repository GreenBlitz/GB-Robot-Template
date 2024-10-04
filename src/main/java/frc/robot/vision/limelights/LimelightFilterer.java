package frc.robot.vision.limelights;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.poseestimator.IPoseEstimator;
import frc.robot.poseestimator.PoseArrayEntryValue;
import frc.robot.poseestimator.PoseEstimationMath;
import frc.robot.poseestimator.observations.VisionObservation;
import frc.utils.Conversions;
import frc.utils.GBSubsystem;
import org.littletonrobotics.junction.Logger;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class LimelightFilterer extends GBSubsystem {

	private final MultiLimelightsRawData multiLimelightsRawData;
	private final IPoseEstimator poseEstimator;
	private double lastSuccessfulObservationTime;

	public LimelightFilterer(LimelightFiltererConfig config, IPoseEstimator poseEstimator) {
		super(config.logPath());

		this.multiLimelightsRawData = new MultiLimelightsRawData(config.limelightsNames(), config.hardwareLogPath());
		this.poseEstimator = poseEstimator;
		this.lastSuccessfulObservationTime = Logger.getRealTimestamp() / 1.0e6;
	}

	public void updateGyroAngles(GyroAngleValues gyroAnglesValues) {
		multiLimelightsRawData.updateGyroAngles(gyroAnglesValues);
	}

	public List<VisionObservation> getFilteredVisionObservations() {
		ArrayList<VisionObservation> estimates = new ArrayList<>();

		for (LimelightRawData limelightRawData : multiLimelightsRawData.getAllAvailableLimelightData()) {
			if (keepLimelightData(limelightRawData)) {
				estimates.add(rawDataToObservation(limelightRawData));
			}
		}
		if (!estimates.isEmpty()) {
			lastSuccessfulObservationTime = Conversions.microSecondsToSeconds(Logger.getRealTimestamp());
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
		Optional<Pose2d> estimatedPoseAtTimeStamp = poseEstimator.getEstimatedPoseAtTimeStamp(limelightRawData.timestamp());
		double[] standardTransformDeviations = estimatedPoseAtTimeStamp
			.map(pose2d -> PoseEstimationMath.calculateStandardDeviationOfPose(limelightRawData, pose2d))
			.orElseGet(
				() -> PoseEstimationMath.calculateStandardDeviationOfPose(limelightRawData, poseEstimator.getEstimatedPose())
			);
		double[] standardDeviations = new double[] {
			standardTransformDeviations[PoseArrayEntryValue.X_VALUE.getEntryValue()],
			standardTransformDeviations[PoseArrayEntryValue.Y_VALUE.getEntryValue()],
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

	private void correctPoseEstimation() {
		boolean hasTooMuchTimePassed = Logger.getRealTimestamp() / 1.0e6 - lastSuccessfulObservationTime
			> LimeLightConstants.TIME_TO_FIX_POSE_ESTIMATION;
		List<VisionObservation> estimates = getFilteredVisionObservations();
		if (hasTooMuchTimePassed && !estimates.isEmpty()) {
			Optional<Pose2d> visionPose = poseEstimator.getVisionPose();
			visionPose.ifPresent(poseEstimator::resetPose);
		}
		lastSuccessfulObservationTime = Logger.getRealTimestamp() / 1.0e6;
	}

	@Override
	protected void subsystemPeriodic() {
		correctPoseEstimation();
	}

}
