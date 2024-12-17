package frc.robot.vision.multivisionsources;

import frc.robot.subsystems.GBSubsystem;
import frc.robot.vision.VisionConstants;
import frc.robot.vision.rawdata.VisionData;
import frc.robot.vision.sources.VisionSource;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;

public class MultiVisionSources<ReturnType extends VisionData> extends GBSubsystem {

	private final List<VisionSource<ReturnType>> visionSources;

	@SafeVarargs
	public MultiVisionSources(String logPath, VisionSource<ReturnType>... visionSources) {
		super(logPath);
		this.visionSources = List.of(visionSources);
	}

	public MultiVisionSources(String logPath, List<VisionSource<ReturnType>> visionSources) {
		super(logPath);
		this.visionSources = visionSources;
	}

	protected List<VisionSource<ReturnType>> getVisionSources() {
		return visionSources;
	}

	protected ArrayList<ReturnType> createMappedCopyOfSources(
		List<VisionSource<ReturnType>> list,
		Function<VisionSource<ReturnType>, Optional<ReturnType>> mapping
	) {
		ArrayList<ReturnType> output = new ArrayList<>();
		list.forEach(visionSource -> {
			visionSource.update();
			Optional<ReturnType> observation = mapping.apply(visionSource);
			observation.ifPresent(output::add);
		});
		return output;
	}

	public ArrayList<ReturnType> getUnfilteredVisionData() {
		return createMappedCopyOfSources(visionSources, VisionSource::getVisionData);
	}

	public ArrayList<ReturnType> getFilteredVisionData() {
		return createMappedCopyOfSources(visionSources, VisionSource::getFilteredData);
	}

	private static <ReturnType extends VisionData> void logPoses(String logPath, String logPathAddition, List<ReturnType> observations) {
		for (int i = 0; i < observations.size(); i++) {
			Logger.recordOutput(logPath + logPathAddition + i, observations.get(i).getEstimatedPose());
		}
	}

	private void log() {
		logPoses(getLogPath(), VisionConstants.FILTERED_DATA_LOGPATH_ADDITION, getFilteredVisionData());
		logPoses(getLogPath(), VisionConstants.NON_FILTERED_DATA_LOGPATH_ADDITION, getUnfilteredVisionData());
	}

	@Override
	protected void subsystemPeriodic() {
		log();
	}

}
