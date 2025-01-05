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
import java.util.function.BiFunction;
import java.util.function.Function;

public class MultiVisionSources<ReturnType extends VisionData> {

	protected final String logPath;
	protected final List<VisionSource<ReturnType>> visionSources;

	@SafeVarargs
	public MultiVisionSources(String logPath, VisionSource<ReturnType>... visionSources) {
		this(logPath, Arrays.asList(visionSources));
	}

	public MultiVisionSources(String logPath, List<VisionSource<ReturnType>> visionSources) {
		this.logPath = logPath;
		this.visionSources = visionSources;
	}

	public ArrayList<ReturnType> getUnfilteredVisionData() {
		return createMappedCopyOfSources(visionSources, VisionSource::getVisionData);
	}

	public ArrayList<ReturnType> getFilteredVisionData() {
		return createMappedCopyOfSources(visionSources, VisionSource::getFilteredVisionData);
	}

	public void applyOnExistingFiltersWithNewFilter(
		BiFunction<Filter<ReturnType>, Filter<ReturnType>, Filter<ReturnType>> applicationFunction,
		Filter<ReturnType> filterToApplyWith
	) {
		for (VisionSource<ReturnType> visionSource : visionSources) {
			visionSource.applyOnExistingFilterWithNewFilter(applicationFunction, filterToApplyWith);
		}
	}

	public void clearFilters() {
		for (VisionSource<ReturnType> visionSource : visionSources) {
			visionSource.clearFilter();
		}
	}

	public void log() {
		logPoses(logPath + VisionConstants.FILTERED_DATA_LOGPATH_ADDITION, getFilteredVisionData());
		logPoses(logPath + VisionConstants.NON_FILTERED_DATA_LOGPATH_ADDITION, getUnfilteredVisionData());
	}

	protected static <ReturnType extends VisionData> ArrayList<ReturnType> createMappedCopyOfSources(
		List<VisionSource<ReturnType>> list,
		Function<VisionSource<ReturnType>, Optional<ReturnType>> mapping
	) {
		ArrayList<ReturnType> output = new ArrayList<>();
		for (VisionSource<ReturnType> visionSource : list) {
			visionSource.update();
			Optional<ReturnType> observation = mapping.apply(visionSource);
			observation.ifPresent(output::add);
		}
		return output;
	}

	protected static <ReturnType extends VisionData> void logPoses(String logPath, List<ReturnType> observations) {
		for (ReturnType observation : observations) {
			Logger.recordOutput(logPath + observation.getSourceName(), observation.getEstimatedPose());
		}
	}

}
