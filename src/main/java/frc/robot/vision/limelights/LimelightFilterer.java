package frc.robot.vision.limelights;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.poseestimator.PoseEstimationMath;
import frc.robot.poseestimator.observations.VisionObservation;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.vision.aprilTags.AprilTagFilters;
import frc.robot.vision.VisionRawData;
import frc.utils.Conversions;
import org.littletonrobotics.junction.Logger;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

public class LimelightFilterer extends GBSubsystem implements ILimelightFilterer {

	private final MultiLimelights multiLimelights;
	private double lastSuccessfulObservationTime;
	private Function<Double, Pose2d> getEstimatedPoseAtTimestamp;

	public LimelightFilterer(String logPath, MultiLimelights multiLimelights) {
		super(logPath);

		this.multiLimelights = multiLimelights;
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

		for (VisionRawData visionData : multiLimelights.getAllAvailableLimelightData()) {
			if (
				AprilTagFilters.keepLimelightData(
					visionData,
					getEstimatedPoseAtTimestamp.apply(Conversions.microSecondsToSeconds(Logger.getRealTimestamp())),
					LimeLightConstants.DEFAULT_LIMELIGHT_FILTERS_TOLERANCES
				)
			) {
				estimates.add(
					PoseEstimationMath.rawDataToObservation(
						visionData,
						getEstimatedPoseAtTimestamp.apply(Conversions.microSecondsToSeconds(Logger.getRealTimestamp()))
					)
				);
			}
		}
		if (!estimates.isEmpty()) {
			lastSuccessfulObservationTime = Conversions.microSecondsToSeconds(Logger.getRealTimestamp());
		}
		return estimates;
	}

	@Override
	public List<VisionObservation> getAllAvailableRawData() {
		ArrayList<VisionObservation> estimates = new ArrayList<>();

		for (VisionRawData visionData : multiLimelights.getAllAvailableLimelightData()) {
			estimates.add(
				PoseEstimationMath.rawDataToObservation(
					visionData,
					getEstimatedPoseAtTimestamp.apply(Conversions.microSecondsToSeconds(Logger.getRealTimestamp()))
				)
			);
		}

		return estimates;
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
	public boolean isPoseEstimationCorrect() {
		boolean hasTooMuchTimePassed = Conversions.microSecondsToSeconds(Logger.getRealTimestamp()) - lastSuccessfulObservationTime
			> LimeLightConstants.TIME_TO_FIX_POSE_ESTIMATION_SECONDS;
		List<VisionObservation> estimates = getAllAvailableRawData();
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
