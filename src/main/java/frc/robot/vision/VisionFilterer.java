package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.poseestimator.PoseArrayEntryValue;
import frc.robot.poseestimator.PoseEstimationMath;
import frc.robot.poseestimator.observations.VisionObservation;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.vision.rawdata.RawVisionData;
import frc.robot.vision.multivisionsources.MultiVisionSources;
import frc.robot.vision.sources.VisionSource;
import frc.utils.time.TimeUtils;
import org.littletonrobotics.junction.Logger;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

public class VisionFilterer<T extends VisionSource<? extends RawVisionData>> extends GBSubsystem {

	private final MultiVisionSources<T> multiVisionSources;
	private final VisionFiltererConfig config;
	private final Function<Double, Pose2d> getEstimatedPoseAtTimestamp;

	public VisionFilterer(
		VisionFiltererConfig config,
		MultiVisionSources<T> multiVisionSources,
		Function<Double, Pose2d> getEstimatedPoseAtTimestamp
	) {
		super(config.logPath());

		this.multiVisionSources = multiVisionSources;
		this.config = config;
		this.getEstimatedPoseAtTimestamp = getEstimatedPoseAtTimestamp;
	}

	public List<VisionObservation> getFilteredVisionObservations() {
		ArrayList<VisionObservation> estimates = new ArrayList<>();

		for (RawVisionData rawVisionData : multiVisionSources.getAllAvailablePoseData()) {
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
			VisionConstants.VISION_ANGLE_STANDARD_DEVIATION};

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

	private void logFilteredOutRawData(RawVisionData rawVisionData) {
		Logger.recordOutput(super.getLogPath() + VisionConstants.FILTERED_OUT_RAW_DATA_LOGPATH_ADDITION, rawVisionData.estimatedPose());
	}

	@Override
	public void subsystemPeriodic() {
		logEstimatedPositions();
		logNonFilteredEstimatedPositions();
	}

}
