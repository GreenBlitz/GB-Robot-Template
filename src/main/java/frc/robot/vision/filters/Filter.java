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

	public boolean applyFilter(T data) {
		return filteringFunction.apply(data);
	}

	public Filter<T> and(Filter<T> secondFilter) {
		return new Filter<>(data -> secondFilter.applyFilter(data) && applyFilter(data));
	}

	public Filter<T> or(Filter<T> anotherFilter) {
		return new Filter<>(data -> anotherFilter.applyFilter(data) || applyFilter(data));
	}

}
