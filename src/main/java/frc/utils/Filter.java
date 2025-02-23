package frc.utils;

import java.util.function.Function;

public class Filter<T> {

	private final Function<T, Boolean> filter;

	public Filter(Function<T, Boolean> filter) {
		this.filter = filter;
	}

	public static <T> Filter<T> nonFilteringFilter() {
		return new Filter<>(data -> true);
	}

	public boolean apply(T data) {
		return filter.apply(data);
	}

	public Filter<T> not() {
		return new Filter<>(data -> !apply(data));
	}

	public <E extends T> Filter<E> and(Filter<E> otherFilter) {
		return new Filter<>(data -> apply(data) && otherFilter.apply(data));
	}

	public <E extends T> Filter<E> or(Filter<E> otherFilter) {
		return new Filter<>(data -> apply(data) || otherFilter.apply(data));
	}

	public <E extends T> Filter<E> xor(Filter<E> otherFilter) {
		return new Filter<>(data -> apply(data) ^ otherFilter.apply(data));
	}

	public <E extends T> Filter<E> implies(Filter<E> otherFilter) {
		return new Filter<>(data -> !apply(data) || otherFilter.apply(data));
	}

	public <E extends T> Filter<E> ifAndOnlyIf(Filter<E> otherFIlter) {
		return new Filter<>(data -> apply(data) == otherFIlter.apply(data));
	}

}
