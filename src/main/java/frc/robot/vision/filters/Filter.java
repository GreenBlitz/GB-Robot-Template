package frc.robot.vision.filters;

import frc.robot.vision.rawdata.RawVisionData;

import java.util.Arrays;
import java.util.function.Function;

public class Filter<T extends RawVisionData> {

	private final Function<T, Boolean> filteringFunction;

	public Filter(Function<T, Boolean> filteringFunction) {
		this.filteringFunction = filteringFunction;
	}

	public boolean doesFilterPasses(T data) {
		return filteringFunction.apply(data);
	}

	public Filter<T> andThen(Filter<T> anotherFilter) {
		return new Filter<>((T data) -> anotherFilter.doesFilterPasses(data) && doesFilterPasses(data));
	}

	public Filter<T> orThen(Filter<T> anotherFilter) {
		return new Filter<>((T data) -> anotherFilter.doesFilterPasses(data) || doesFilterPasses(data));
	}

	@SafeVarargs
	public static<T extends RawVisionData> Filter<T> combineFilters(Filter<T>... filters) {
		return new Filter<>((T data) -> Arrays.stream(filters).allMatch((Filter<T> filer) -> filer.doesFilterPasses(data)));
	}

}
