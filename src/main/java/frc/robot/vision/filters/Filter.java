package frc.robot.vision.filters;

import java.util.function.Function;

public class Filter<T> {

	private final Function<T, Boolean> filterer;

	public Filter(Function<T, Boolean> filterer) {
		this.filterer = filterer;
	}

	public boolean apply(T data) {
		return filterer.apply(data);
	}

	public Filter<T> and(Filter<T> otherFilter) {
		return new Filter<>(data -> otherFilter.apply(data) && apply(data));
	}

	public Filter<T> or(Filter<T> otherFilter) {
		return new Filter<>(data -> otherFilter.apply(data) || apply(data));
	}

}
