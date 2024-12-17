package frc.robot.vision.filters;


import frc.robot.vision.rawdata.VisionData;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;
import java.util.function.Function;

public class Filter<T extends VisionData> {

	private final Function<T, Boolean> filteringFunction;

	public Filter(Function<T, Boolean> filteringFunction) {
		this.filteringFunction = filteringFunction;
	}

	public boolean applyFilter(T data) {
		return filteringFunction.apply(data);
	}

	public Filter<T> and(Filter<T> secondFilter) {
		return new Filter<>(data -> secondFilter.applyFilter(data) && applyFilter(data));
	}

	public Filter<T> or(Filter<T> anotherFilter) {
		return new Filter<>(data -> anotherFilter.applyFilter(data) || applyFilter(data));
	}

	public void logFilter(String logPath, T data) {
		Logger.recordOutput(logPath, applyFilter(data));
	}

	@SafeVarargs
	public static <T extends VisionData> Filter<T> combineFilters(Filter<T>... filters) {
		return new Filter<>(data -> Arrays.stream(filters).allMatch(filter -> filter.applyFilter(data)));
	}

	@SafeVarargs
	public static <T extends VisionData> void logFilters(String logPath, T data, Filter<T>... filters) {
		for (int i = 0; i < filters.length; i++) {
			filters[i].logFilter(logPath + "/" + i, data);
		}
	}

}
