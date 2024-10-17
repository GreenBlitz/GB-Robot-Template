package frc.robot.vision.limelights;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.poseestimator.PoseArrayEntryValue;
import frc.robot.poseestimator.PoseEstimationMath;
import frc.robot.poseestimator.observations.VisionObservation;
import frc.robot.subsystems.GBSubsystem;
import frc.utils.time.TimeUtils;
import org.littletonrobotics.junction.Logger;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

public class LimelightFilterer extends GBSubsystem implements ILimelightFilterer {

	private final MultiLimelights multiLimelights;
	private final LimelightFiltererConfig config;
	private Function<Double, Pose2d> getEstimatedPoseAtTimestamp;

	public LimelightFilterer(LimelightFiltererConfig config, MultiLimelights multiLimelights) {
		super(config.logPath());

		this.multiLimelights = multiLimelights;
		this.config = config;
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
			if (LimelightFilters.keepLimelightData(limelightRawData, config.limelightFiltersTolerances(), super.getLogPath())) {
				estimates.add(rawDataToObservation(limelightRawData));
			}
		}
		return estimates;
	}

	@Override
	public List<VisionObservation> getAllAvailableVisionObservations() {
		ArrayList<VisionObservation> estimates = new ArrayList<>();

		for (LimelightRawData limelightRawData : multiLimelights.getAllAvailableLimelightData()) {
			estimates.add(rawDataToObservation(limelightRawData));
		}

		return estimates;
	}

	private VisionObservation rawDataToObservation(LimelightRawData limelightRawData) {
		double[] standardTransformDeviations = PoseEstimationMath
			.calculateStandardDeviationOfPose(limelightRawData, getEstimatedPoseAtTimestamp.apply(TimeUtils.getCurrentTimeSeconds()));
		double[] standardDeviations = new double[] {
			standardTransformDeviations[PoseArrayEntryValue.X_VALUE.getEntryValue()],
			standardTransformDeviations[PoseArrayEntryValue.Y_VALUE.getEntryValue()],
			LimeLightConstants.VISION_STANDARD_DEVIATION_ANGLES};

		return new VisionObservation(limelightRawData.estimatedPose().toPose2d(), standardDeviations, limelightRawData.timestamp());
	}

	private void logEstimatedPositions() {
		List<VisionObservation> observations = getAllAvailableVisionObservations();

		for (int i = 0; i < observations.size(); i++) {
			Logger.recordOutput(super.getLogPath() + LimeLightConstants.ESTIMATION_LOGPATH_PREFIX + i, observations.get(i).robotPose());
		}
	}

	private void logAprilTagHeights() {
		List<LimelightRawData> observations = multiLimelights.getAllAvailableLimelightData();

		for (int i = 0; i < observations.size(); i++) {
			Logger
				.recordOutput(super.getLogPath() + LimeLightConstants.APRIL_TAG_HEIGHT_LOGPATH_PREFIX + i, observations.get(i).aprilTagHeight());
		}
	}

	@Override
	public void subsystemPeriodic() {
		logEstimatedPositions();
	}

}
