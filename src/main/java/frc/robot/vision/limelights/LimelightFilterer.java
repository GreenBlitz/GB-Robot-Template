package frc.robot.vision.limelights;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.poseestimator.IPoseEstimator;
import frc.robot.poseestimator.PoseArrayEntryValue;
import frc.robot.poseestimator.PoseEstimationMath;
import frc.robot.poseestimator.observations.VisionObservation;
import frc.robot.subsystems.GBSubsystem;
import frc.utils.Conversions;
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
		this.lastSuccessfulObservationTime = Conversions.microSecondsToSeconds(Logger.getRealTimestamp());
	}

	public void updateGyroAngles(GyroAngleValues gyroAngleValues) {
		multiLimelightsRawData.updateGyroAngles(gyroAngleValues);
	}

	public List<VisionObservation> getFilteredVisionObservations() {
		ArrayList<VisionObservation> estimates = new ArrayList<>();

		for (LimelightRawData limelightRawData : multiLimelightsRawData.getAllAvailableLimelightData()) {
			if (LimelightFilters.keepLimelightData(limelightRawData, poseEstimator.getEstimatedPose())) {
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
			.orElseGet(() -> PoseEstimationMath.calculateStandardDeviationOfPose(limelightRawData, poseEstimator.getEstimatedPose()));
		double[] standardDeviations = new double[] {
			standardTransformDeviations[PoseArrayEntryValue.X_VALUE.getEntryValue()],
			standardTransformDeviations[PoseArrayEntryValue.Y_VALUE.getEntryValue()],
			LimeLightConstants.STANDARD_DEVIATION_VISION_DEGREES};

		return new VisionObservation(limelightRawData.estimatedPose().toPose2d(), standardDeviations, limelightRawData.timestamp());
	}

	public void logEstimatedPositions() {
		List<VisionObservation> observations = getFilteredVisionObservations();

		for (int i = 0; i < observations.size(); i++) {
			Logger.recordOutput(
				super.getLogPath() + LimeLightConstants.ESTIMATION_LOGPATH_PREFIX + i + "Time" + observations.get(i).timestamp(),
				observations.get(i).robotPose()
			);
		}
	}

	private void correctPoseEstimation() {
		boolean hasTooMuchTimePassed = Conversions.microSecondsToSeconds(Logger.getRealTimestamp()) - lastSuccessfulObservationTime
			> LimeLightConstants.TIME_TO_FIX_POSE_ESTIMATION_SECONDS;
		List<VisionObservation> estimates = getAllAvailableLimelightData();
		if (hasTooMuchTimePassed && !estimates.isEmpty()) {
			Optional<Pose2d> visionPose = poseEstimator.getVisionPose();
			visionPose.ifPresent(poseEstimator::resetPose);
		}
		lastSuccessfulObservationTime = Conversions.microSecondsToSeconds(Logger.getRealTimestamp());
	}

	@Override
	protected void subsystemPeriodic() {
		correctPoseEstimation();
	}

}
