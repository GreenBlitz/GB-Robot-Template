package frc.robot.vision.multivisionsources;

import frc.robot.poseestimator.observations.IRobotPoseVisionObservation;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.vision.VisionConstants;
import frc.robot.vision.rawdata.RawVisionData;
import frc.robot.vision.sources.VisionSource;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;

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

	private <ReturnType> ArrayList<ReturnType> createMappedCopyOfSources(List<VisionSourceType> list, Function<Optional<? extends RawVisionData>, Optional<ReturnType>> mapping) {
		ArrayList<ReturnType> output = new ArrayList<>();
		list.forEach(visionSource -> {
			visionSource.update();
			Optional<ReturnType> observation = mapping.apply(visionSource.getRawVisionData());
			observation.ifPresent(output::add);
		});
		return output;
	}

	public ArrayList<IRobotPoseVisionObservation> getUnfilteredVisionObservation() {
		return createMappedCopyOfSources(visionSources, this::convertToOptionalObservation);
	}

	public ArrayList<IRobotPoseVisionObservation> getFilteredVisionObservations() {
		return createMappedCopyOfSources(visionSources, (rawVisionData -> {}));;
	}

	/**
	 * Returns the same optional but extract the object out of the Optional since java doesn't support polymorphism of generics inside optional
	 *
	 * @param optionalRawVisionData: the optional to be converted
	 * @return: new instance that has the same data but java is happier with it
	 */
	private Optional<IRobotPoseVisionObservation> convertToOptionalObservation(Optional<? extends RawVisionData> optionalRawVisionData) {
		if (optionalRawVisionData.isPresent()) {
			return Optional.of(optionalRawVisionData.get());
		}
		return Optional.empty();
	}

	private static void logRobotPose(String logPath, String logPathAddition, List<IRobotPoseVisionObservation> observations) {
		for (int i = 0; i < observations.size(); i++) {
			Logger.recordOutput(logPath + logPathAddition + i, observations.get(i).getEstimatedPose());
		}
	}

	private void log() {
		logRobotPose(getLogPath(), VisionConstants.FILTERED_DATA_LOGPATH_ADDITION, getFilteredVisionObservations());
		logRobotPose(getLogPath(), VisionConstants.NON_FILTERED_DATA_LOGPATH_ADDITION, getUnfilteredVisionObservation());
	}

	@Override
	protected void subsystemPeriodic() {
		log();
	}

}
