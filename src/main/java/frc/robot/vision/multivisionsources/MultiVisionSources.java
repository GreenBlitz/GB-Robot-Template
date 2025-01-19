package frc.robot.vision.multivisionsources;

import frc.constants.VisionConstants;
import frc.robot.vision.data.VisionData;
import frc.robot.vision.sources.VisionSource;
import frc.utils.Filter;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;

public class MultiVisionSources<T extends VisionData> {

	protected final String logPath;
	protected final List<VisionSource<T>> visionSources;

	@SafeVarargs
	public MultiVisionSources(String logPath, VisionSource<T>... visionSources) {
		this(logPath, Arrays.asList(visionSources));
	}

	public MultiVisionSources(String logPath, List<VisionSource<T>> visionSources) {
		this.logPath = logPath;
		this.visionSources = visionSources;
	}

	public ArrayList<T> getUnfilteredVisionData() {
		return createMappedCopyOfSources(visionSources, VisionSource::getVisionData);
	}

	public ArrayList<T> getFilteredVisionData() {
		return createMappedCopyOfSources(visionSources, VisionSource::getFilteredVisionData);
	}

	public void applyFunctionOnAllFilters(Function<Filter<T>, Filter<T>> filterChangingFunction) {
		for (VisionSource<T> visionSource : visionSources) {
			visionSource.applyFunctionOnFilter(filterChangingFunction);
		}
	}

	public void clearFilters() {
		for (VisionSource<T> visionSource : visionSources) {
			visionSource.clearFilter();
		}
	}

	public void log() {
		logPoses(logPath + VisionConstants.FILTERED_DATA_LOGPATH_ADDITION, getFilteredVisionData());
		logPoses(logPath + VisionConstants.NON_FILTERED_DATA_LOGPATH_ADDITION, getUnfilteredVisionData());
	}

	protected static <T extends VisionData> ArrayList<T> createMappedCopyOfSources(
		List<VisionSource<T>> list,
		Function<VisionSource<T>, Optional<T>> mapping
	) {
		ArrayList<T> output = new ArrayList<>();
		for (VisionSource<T> visionSource : list) {
			visionSource.update();
			Optional<T> observation = mapping.apply(visionSource);
			observation.ifPresent(output::add);
		}
		return output;
	}

	protected static <T extends VisionData> void logPoses(String logPath, List<T> observations) {
		for (T observation : observations) {
			Logger.recordOutput(logPath + observation.getSourceName(), observation.getEstimatedPose());
		}
	}

}
