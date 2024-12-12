package frc.robot.vision.multivisionsources;

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

	private <ReturnType> ArrayList<ReturnType> createMappedCopyOfSources(
		List<VisionSourceType> list,
		Function<Optional<? extends RawVisionData>, Optional<ReturnType>> mapping
	) {
		ArrayList<ReturnType> output = new ArrayList<>();
		list.forEach(visionSource -> {
			visionSource.update();
			Optional<ReturnType> observation = mapping.apply(visionSource.getRawVisionData());
			observation.ifPresent(output::add);
		});
		return output;
	}

	public ArrayList<? extends RawVisionData> getUnfilteredVisionData() {
		return createMappedCopyOfSources(visionSources, (data) -> data);
	}

	public ArrayList<? extends RawVisionData> getFilteredVisionData() {
		return createMappedCopyOfSources(visionSources, (rawVisionData -> {
			if (rawVisionData.isPresent()) {
				if (!rawVisionData.get().getIsDataValid()) {
					return Optional.empty();
				}
				return rawVisionData;
			}
			return Optional.empty();
		}));
	}

	private static void logRobotPose(String logPath, String logPathAddition, List<? extends RawVisionData> observations) {
		for (int i = 0; i < observations.size(); i++) {
			Logger.recordOutput(logPath + logPathAddition + i, observations.get(i).getEstimatedPose());
		}
	}

	private void log() {
		logRobotPose(getLogPath(), VisionConstants.FILTERED_DATA_LOGPATH_ADDITION, getFilteredVisionData());
		logRobotPose(getLogPath(), VisionConstants.NON_FILTERED_DATA_LOGPATH_ADDITION, getUnfilteredVisionData());
	}

	@Override
	protected void subsystemPeriodic() {
		log();
	}

}
