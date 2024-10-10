package frc.robot.vision.limelights;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
	private final LimelightFiltererConfig config;
	private double lastSuccessfulObservationTime;
	private Function<Double, Pose2d> getEstimatedPoseAtTimestamp;

	public LimelightFilterer(LimelightFiltererConfig config, MultiLimelights multiLimelights) {
		super(config.logPath());

		this.multiLimelights = multiLimelights;
		this.config = config;
		this.lastSuccessfulObservationTime = Conversions.microSecondsToSeconds(Logger.getRealTimestamp());
	}

	@Override
	public List<Rotation2d> getAllRobotHeadingEstimations() {
		return multiLimelights.getAllRobotHeadingEstimations();
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
					config.aprilTagHeightMeters(),
					LimeLightConstants.DEFAULT_LIMELIGHT_FILTERS_TOLERANCES,
					super.getLogPath()
				)
			) {;
				estimates.add(rawDataToObservation(limelightRawData));
			}
		}
		
		for (int i = 0; i < estimates.size(); i++) {
			Logger.recordOutput("estimateee" + i , estimates.get(i).robotPose());
		}
		
		
		if (!estimates.isEmpty()) {
			lastSuccessfulObservationTime = Conversions.microSecondsToSeconds(Logger.getRealTimestamp());
		}
		return estimates;
	}

	@Override
	public List<VisionObservation> getAllAvailableLimelightRawData() {
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
		List<VisionObservation> observations = getAllAvailableLimelightRawData();

		for (int i = 0; i < observations.size(); i++) {
			Logger.recordOutput(
				super.getLogPath() + LimeLightConstants.ESTIMATION_LOGPATH_PREFIX + i,
				observations.get(i).robotPose()
			);
		}
	}

	@Override
	public boolean isPoseEstimationCorrect() {
		boolean hasTooMuchTimePassed = Conversions.microSecondsToSeconds(Logger.getRealTimestamp()) - lastSuccessfulObservationTime
			> LimeLightConstants.TIME_TO_FIX_POSE_ESTIMATION_SECONDS;
		List<VisionObservation> estimates = getAllAvailableLimelightRawData();
		if (hasTooMuchTimePassed && !estimates.isEmpty()) {
			lastSuccessfulObservationTime = Conversions.microSecondsToSeconds(Logger.getRealTimestamp());
			return false;
		}
		return true;
	}

	@Override
	public void subsystemPeriodic() {
		logEstimatedPositions();
	}

}
