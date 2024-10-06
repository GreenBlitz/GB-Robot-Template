package frc.robot.vision.limelights;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.poseestimator.PoseArrayEntryValue;
import frc.robot.poseestimator.PoseEstimationMath;
import frc.robot.poseestimator.observations.VisionObservation;
import frc.robot.subsystems.GBSubsystem;
import frc.utils.Conversions;
import org.littletonrobotics.junction.Logger;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

public class LimelightFilterer extends GBSubsystem implements ILimelightFilterer {

	private final MultiLimelights multiLimelights;
	private double lastSuccessfulObservationTime;
	private Function<Double, Pose2d> getEstimatedPoseAtTimestamp;

	public LimelightFilterer(LimelightFiltererConfig config) {
		super(config.logPath());

		this.multiLimelights = new MultiLimelights(config.limelightsNames(), config.hardwareLogPath());
		this.lastSuccessfulObservationTime = Conversions.microSecondsToSeconds(Logger.getRealTimestamp());
	}

	@Override
	public void setEstimatedPoseAtTimestampFunction(Function<Double, Pose2d> getEstimatedPoseAtTimestamp) {
		this.getEstimatedPoseAtTimestamp = getEstimatedPoseAtTimestamp;
	}

	@Override
	public void updateGyroAngles(GyroAngleValues gyroAngleValues) {
		multiLimelights.updateGyroAngles(gyroAngleValues);
	}

	@Override
	public List<VisionObservation> getFilteredVisionObservations() {
		ArrayList<VisionObservation> estimates = new ArrayList<>();

		for (LimelightRawData limelightRawData : multiLimelights.getAllAvailableLimelightData()) {
			if (
				LimelightFilters.keepLimelightData(
					limelightRawData,
					getEstimatedPoseAtTimestamp.apply(Conversions.microSecondsToSeconds(Logger.getRealTimestamp())),
					LimeLightConstants.DEFAULT_LIMELIGHT_FILTERS_TOLERANCES
				)
			) {
				estimates.add(rawDataToObservation(limelightRawData));
			}
		}
		if (!estimates.isEmpty()) {
			lastSuccessfulObservationTime = Conversions.microSecondsToSeconds(Logger.getRealTimestamp());
		}
		return estimates;
	}

	@Override
	public List<VisionObservation> getAllAvailableLimelightData() {
		ArrayList<VisionObservation> estimates = new ArrayList<>();

		for (LimelightRawData limelightRawData : multiLimelights.getAllAvailableLimelightData()) {
			estimates.add(rawDataToObservation(limelightRawData));
		}

		return estimates;
	}

	private VisionObservation rawDataToObservation(LimelightRawData limelightRawData) {
		double[] standardTransformDeviations = PoseEstimationMath.calculateStandardDeviationOfPose(
			limelightRawData,
			getEstimatedPoseAtTimestamp.apply(Conversions.microSecondsToSeconds(Logger.getRealTimestamp()))
		);
		double[] standardDeviations = new double[] {
			standardTransformDeviations[PoseArrayEntryValue.X_VALUE.getEntryValue()],
			standardTransformDeviations[PoseArrayEntryValue.Y_VALUE.getEntryValue()],
			LimeLightConstants.VISION_STANDARD_DEVIATION_ANGLES};

		return new VisionObservation(limelightRawData.estimatedPose().toPose2d(), standardDeviations, limelightRawData.timestamp());
	}

	private void logEstimatedPositions() {
		List<VisionObservation> observations = getFilteredVisionObservations();

		for (int i = 0; i < observations.size(); i++) {
			Logger.recordOutput(
				super.getLogPath() + LimeLightConstants.ESTIMATION_LOGPATH_PREFIX + i + "Time" + observations.get(i).timestamp(),
				observations.get(i).robotPose()
			);
		}
	}

	@Override
	public boolean correctPoseEstimation() {
		boolean hasTooMuchTimePassed = Conversions.microSecondsToSeconds(Logger.getRealTimestamp()) - lastSuccessfulObservationTime
			> LimeLightConstants.TIME_TO_FIX_POSE_ESTIMATION_SECONDS;
		List<VisionObservation> estimates = getAllAvailableLimelightData();
		if (hasTooMuchTimePassed && !estimates.isEmpty()) {
			lastSuccessfulObservationTime = Conversions.microSecondsToSeconds(Logger.getRealTimestamp());
			return true;
		}
		return false;
	}

	@Override
	public void subsystemPeriodic() {
		logEstimatedPositions();
	}

}
