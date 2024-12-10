package frc.robot.vision.multivisionsources;

import frc.robot.poseestimator.observations.VisionRobotPoseObservation;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.vision.VisionConstants;
import frc.robot.vision.rawdata.RawVisionData;
import frc.robot.vision.sources.VisionSource;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class MultiVisionSources<T extends VisionSource<? extends RawVisionData>> extends GBSubsystem {

	private final List<T> visionSources;

	@SafeVarargs
	public MultiVisionSources(String logPath, T... visionSources) {
		super(logPath);
		this.visionSources = List.of(visionSources);
	}

	public MultiVisionSources(String logPath, List<T> visionSources) {
		super(logPath);
		this.visionSources = visionSources;
	}

	protected List<T> getVisionSources() {
		return visionSources;
	}

	public ArrayList<VisionRobotPoseObservation> getUnFilteredVisionObservation() {
		ArrayList<VisionRobotPoseObservation> rawPoseData = new ArrayList<>();
		visionSources.forEach(visionSource -> {
			visionSource.updateEstimation();
			Optional<VisionRobotPoseObservation> observation = convertToOptionalObservation(visionSource.getRawVisionEstimation());
			observation.ifPresent(rawPoseData::add);
		});
		return rawPoseData;
	}

	public ArrayList<VisionRobotPoseObservation> getFilteredVisionObservations() {
		ArrayList<VisionRobotPoseObservation> estimates = new ArrayList<>();

		for (VisionSource<? extends RawVisionData> visionSource : visionSources) {
			if (!visionSource.shallBeFiltered()) {
				Optional<VisionRobotPoseObservation> observation = convertToOptionalObservation(visionSource.getRawVisionEstimation());
				observation.ifPresent(estimates::add);
			}
		}
		return estimates;
	}

	/**
	 * Returns the same optional but extract the object out of the Optional since java doesn't support polymorphism of generics inside optional
	 *
	 * @param optionalRawVisionData: the optional to be converted
	 * @return: new instance that has the same data but java is happier with it
	 */
	private Optional<VisionRobotPoseObservation> convertToOptionalObservation(Optional<? extends RawVisionData> optionalRawVisionData) {
		if (optionalRawVisionData.isPresent()) {
			return Optional.of(optionalRawVisionData.get());
		}
		return Optional.empty();
	}

	private static void logRobotPose(String logPath, String logPathAddition, List<VisionRobotPoseObservation> observations) {
		for (int i = 0; i < observations.size(); i++) {
			Logger.recordOutput(logPath + logPathAddition + i, observations.get(i).getEstimatedPose());
		}
	}

	private void logOutputs() {
		logRobotPose(super.getLogPath(), VisionConstants.FILTERED_ESTIMATION_LOGPATH_ADDITION, getFilteredVisionObservations());
		logRobotPose(super.getLogPath(), VisionConstants.NON_FILTERED_ESTIMATION_LOGPATH_ADDITION, getUnFilteredVisionObservation());
	}

	@Override
	protected void subsystemPeriodic() {
		logOutputs();
	}

}
