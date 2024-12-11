package frc.robot.vision.filters;

import frc.robot.vision.rawdata.RawVisionData;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;
import java.util.function.Function;

public class Filter<T extends RawVisionData> {

	private final Function<T, Boolean> filteringFunction;

	public Filter(Function<T, Boolean> filteringFunction) {
		this.filteringFunction = filteringFunction;
	}

	public boolean doesFilterPass(T data) {
		return filteringFunction.apply(data);
	}

	public Filter<T> andThen(Filter<T> anotherFilter) {
		return new Filter<>((T data) -> anotherFilter.doesFilterPass(data) && doesFilterPass(data));
	}

	public Filter<T> or(Filter<T> anotherFilter) {
		return new Filter<>((T data) -> anotherFilter.doesFilterPass(data) || doesFilterPass(data));
	}

	public void logFilterStatus(String logPath, T data) {
		Logger.recordOutput(logPath, doesFilterPass(data));
	}

	@SafeVarargs
	public static <T extends RawVisionData> Filter<T> combineFilters(Filter<T>... filters) {
		return new Filter<>((T data) -> Arrays.stream(filters).allMatch((Filter<T> filer) -> filer.doesFilterPass(data)));
	}

	@SafeVarargs
	public static <T extends RawVisionData> void logFiltersStatus(String logPath, T data, Filter<T>... filters) {
		for (int i = 0; i < filters.length; i++) {
			filters[i].logFilterStatus(logPath + "/" + i, data);
		}
	}

}
