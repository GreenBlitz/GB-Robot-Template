package frc.robot.vision.filters;

import frc.robot.vision.rawdata.IRawVisionData;
import frc.robot.vision.rawdata.RawVisionData;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;
import java.util.function.Function;

public class Filter {

	private final Function<IRawVisionData, Boolean> filteringFunction;

	public Filter(Function<IRawVisionData, Boolean> filteringFunction) {
		this.filteringFunction = filteringFunction;
	}

	public boolean doesFilterPasses(IRawVisionData data) {
		return filteringFunction.apply(data);
	}

	public Filter andThen(Filter anotherFilter) {
		return new Filter((IRawVisionData data) -> anotherFilter.doesFilterPasses(data) && doesFilterPasses(data));
	}

	public Filter orThen(Filter anotherFilter) {
		return new Filter((IRawVisionData data) -> anotherFilter.doesFilterPasses(data) || doesFilterPasses(data));
	}

	public void logFilterState(String logPath, IRawVisionData data) {
		Logger.recordOutput(logPath, doesFilterPasses(data));
	}

	@SafeVarargs
	public static <T extends RawVisionData> Filter combineFilters(Filter... filters) {
		return new Filter((IRawVisionData data) -> Arrays.stream(filters).allMatch((Filter filer) -> filer.doesFilterPasses(data)));
	}

	@SafeVarargs
	public static <T extends RawVisionData> void logFiltersState(String logPath, IRawVisionData data, Filter... filters) {
		for (int i = 0; i < filters.length; i++) {
			filters[i].logFilterState(logPath + "/" + i, data);
		}
	}

}
