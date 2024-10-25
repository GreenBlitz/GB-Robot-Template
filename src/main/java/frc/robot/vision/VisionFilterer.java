package frc.robot.vision;

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

public class VisionFilterer extends GBSubsystem implements IVisionFilterer {

	private final MultiVisionSources multiVisionSources;
	private final VisionFiltererConfig config;
	private Function<Double, Pose2d> getEstimatedPoseAtTimestamp;

	public VisionFilterer(VisionFiltererConfig config, MultiVisionSources multiVisionSources) {
		super(config.logPath());

		this.multiVisionSources = multiVisionSources;
		this.config = config;
	}

	@Override
	public List<Rotation2d> getAllRobotHeadingEstimations() {
		return multiVisionSources.getAllRobotHeadingEstimations();
	}

	@Override
	public void setEstimatedPoseAtTimestampFunction(Function<Double, Pose2d> getEstimatedPoseAtTimestamp) {
		this.getEstimatedPoseAtTimestamp = getEstimatedPoseAtTimestamp;
	}

	@Override
	public void updateGyroAngles(GyroAngleValues gyroAngleValues) {
		multiVisionSources.updateGyroAngles(gyroAngleValues);
	}

	@Override
	public List<VisionObservation> getFilteredVisionObservations() {
		ArrayList<VisionObservation> estimates = new ArrayList<>();

		for (RawVisionData rawVisionData : multiVisionSources.getAllAvailablePoseData()) {
			if (VisionFilters.keepVisionData(rawVisionData, config.VisionFiltersTolerances())) {
				estimates.add(rawDataToObservation(rawVisionData));
			}
		}
		return estimates;
	}

	@Override
	public List<VisionObservation> getAllAvailableVisionObservations() {
		ArrayList<VisionObservation> estimates = new ArrayList<>();

		for (RawVisionData rawVisionData : multiVisionSources.getAllAvailablePoseData()) {
			estimates.add(rawDataToObservation(rawVisionData));
		}

		return estimates;
	}

	private VisionObservation rawDataToObservation(RawVisionData rawVisionData) {
		double[] standardTransformDeviations = PoseEstimationMath
			.calculateStandardDeviationOfPose(rawVisionData, getEstimatedPoseAtTimestamp.apply(TimeUtils.getCurrentTimeSeconds()));
		double[] standardDeviations = new double[] {
			standardTransformDeviations[PoseArrayEntryValue.X_VALUE.getEntryValue()],
			standardTransformDeviations[PoseArrayEntryValue.Y_VALUE.getEntryValue()],
			VisionConstants.VISION_STANDARD_DEVIATION_ANGLES};

		return new VisionObservation(rawVisionData.estimatedPose().toPose2d(), standardDeviations, rawVisionData.timestamp());
	}

	private void logEstimatedPositions() {
		List<VisionObservation> observations = getFilteredVisionObservations();

		for (int i = 0; i < observations.size(); i++) {
			Logger.recordOutput(
				super.getLogPath() + VisionConstants.ESTIMATION_LOGPATH_PREFIX + i + "Time" + observations.get(i).timestamp(),
				observations.get(i).robotPose()
			);
		}
	}

	@Override
	public void subsystemPeriodic() {
		logEstimatedPositions();
	}

}
