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

public class MultiVisionSources<VisionSourceType extends VisionSource<? extends RawVisionData>> extends GBSubsystem {

	private final List<VisionSourceType> visionSources;

	@SafeVarargs
	public MultiVisionSources(String logPath, VisionSourceType... visionSources) {
		super(logPath);
		this.visionSources = List.of(visionSources);
	}

	public MultiVisionSources(String logPath, List<VisionSourceType> visionSources) {
		super(logPath);
		this.visionSources = visionSources;
	}

	protected List<VisionSourceType> getVisionSources() {
		return visionSources;
	}

	public ArrayList<VisionRobotPoseObservation> getUnfilteredVisionObservation() {
		ArrayList<VisionRobotPoseObservation> rawPoseData = new ArrayList<>();
		visionSources.forEach(visionSource -> {
			visionSource.update();
			Optional<VisionRobotPoseObservation> observation = convertToOptionalObservation(visionSource.getRawVisionData());
			observation.ifPresent(rawPoseData::add);
		});
		return rawPoseData;
	}

	public ArrayList<VisionRobotPoseObservation> getFilteredVisionObservations() {
		ArrayList<VisionRobotPoseObservation> estimates = new ArrayList<>();

		for (VisionSource<? extends RawVisionData> visionSource : visionSources) {
			if (!visionSource.shouldDataBeFiltered()) {
				Optional<VisionRobotPoseObservation> observation = convertToOptionalObservation(visionSource.getRawVisionData());
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
		logRobotPose(getLogPath(), VisionConstants.FILTERED_DATA_LOGPATH_ADDITION, getFilteredVisionObservations());
		logRobotPose(getLogPath(), VisionConstants.NON_FILTERED_DATA_LOGPATH_ADDITION, getUnfilteredVisionObservation());
	}

	@Override
	protected void subsystemPeriodic() {
		logOutputs();
	}

}
