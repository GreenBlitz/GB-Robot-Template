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

public class VisionFilterer extends GBSubsystem {

	private final MultiVisionSources multiVisionSources;
	private final VisionFiltererConfig config;
	private final Function<Double, Pose2d> getEstimatedPoseAtTimestamp;

	public VisionFilterer(
		VisionFiltererConfig config,
		MultiVisionSources multiVisionSources,
		Function<Double, Pose2d> getEstimatedPoseAtTimestamp
	) {
		super(config.logPath());

		this.multiVisionSources = multiVisionSources;
		this.config = config;
		this.getEstimatedPoseAtTimestamp = getEstimatedPoseAtTimestamp;
	}

	public List<Rotation2d> getAllRobotHeadingEstimations() {
		return multiVisionSources.getAllRobotHeadingEstimations();
	}

	public void updateGyroAngles(GyroAngleValues gyroAngleValues) {
		multiVisionSources.updateGyroAngles(gyroAngleValues);
	}

	public List<VisionObservation> getFilteredVisionObservations() {
		ArrayList<VisionObservation> estimates = new ArrayList<>();

		for (RawVisionAprilTagData rawVisionData : multiVisionSources.getAllAvailablePoseData()) {
			if (config.filters().apply(rawVisionData, config.VisionFiltersTolerances())) {
				estimates.add(rawDataToObservation(rawVisionData));
			} else {
				logFilteredOutRawData(rawVisionData);
			}
		}
		return estimates;
	}

	public List<VisionObservation> getAllAvailableVisionObservations() {
		ArrayList<VisionObservation> estimates = new ArrayList<>();

		for (RawVisionAprilTagData rawVisionData : multiVisionSources.getAllAvailablePoseData()) {
			estimates.add(rawDataToObservation(rawVisionData));
		}

		return estimates;
	}

	private VisionObservation rawDataToObservation(RawVisionAprilTagData rawVisionData) {
		double[] standardTransformDeviations = PoseEstimationMath
			.calculateStandardDeviationOfPose(rawVisionData, getEstimatedPoseAtTimestamp.apply(TimeUtils.getCurrentTimeSeconds()));
		double[] standardDeviations = new double[] {
			standardTransformDeviations[PoseArrayEntryValue.X_VALUE.getEntryValue()],
			standardTransformDeviations[PoseArrayEntryValue.Y_VALUE.getEntryValue()],
			VisionConstants.DEFAULT_VISION_STANDARD_DEVIATIONS[PoseArrayEntryValue.ROTATION_VALUE.getEntryValue()]};

		return new VisionObservation(rawVisionData.estimatedPose().toPose2d(), standardDeviations, rawVisionData.timestamp());
	}

	private void logEstimatedPositions() {
		List<VisionObservation> observations = getFilteredVisionObservations();
		for (int i = 0; i < observations.size(); i++) {
			Logger.recordOutput(super.getLogPath() + VisionConstants.FILTERED_ESTIMATION_LOGPATH_ADDITION + i, observations.get(i).robotPose());
		}
	}

	private void logNonFilteredEstimatedPositions() {
		List<VisionObservation> observations = getAllAvailableVisionObservations();
		for (int i = 0; i < observations.size(); i++) {
			Logger.recordOutput(
				super.getLogPath() + VisionConstants.NON_FILTERED_ESTIMATION_LOGPATH_ADDITION + i,
				observations.get(i).robotPose()
			);
		}
	}

	private void logFilteredOutRawData(RawVisionAprilTagData rawVisionData) {
		Logger.recordOutput(super.getLogPath() + VisionConstants.FILTERED_OUT_RAW_DATA_LOGPATH_ADDITION, rawVisionData.estimatedPose());
	}

	@Override
	public void subsystemPeriodic() {
		logEstimatedPositions();
		logNonFilteredEstimatedPositions();
	}

}
