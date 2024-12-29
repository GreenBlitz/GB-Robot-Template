package frc.utils;

import java.util.function.Function;

public class Filter<T> {

	private final Function<T, Boolean> filter;

	public Filter(Function<T, Boolean> filter) {
		this.filter = filter;
	}

	public boolean apply(T data) {
		return filter.apply(data);
	}

	public Filter<T> and(Filter<T> otherFilter) {
		return new Filter<>(data -> apply(data) && otherFilter.apply(data));
	}

	public Filter<T> or(Filter<T> otherFilter) {
		return new Filter<>(data -> apply(data) || otherFilter.apply(data));
	}

}
