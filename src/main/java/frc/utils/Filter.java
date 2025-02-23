package frc.utils;


public interface Filter<T> {

	static <T> Filter<T> nonFilteringFilter() {
		return data -> true;
	}

	boolean apply(T data);

	default Filter<T> not() {
		return data -> !apply(data);
	}

	default <E extends T> Filter<E> and(Filter<E> otherFilter) {
		return data -> apply(data) && otherFilter.apply(data);
	}

	default <E extends T> Filter<E> or(Filter<E> otherFilter) {
		return data -> apply(data) || otherFilter.apply(data);
	}

	default <E extends T> Filter<E> xor(Filter<E> otherFilter) {
		return data -> apply(data) ^ otherFilter.apply(data);
	}

	default <E extends T> Filter<E> implies(Filter<E> otherFilter) {
		return data -> !apply(data) || otherFilter.apply(data);
	}

	default <E extends T> Filter<E> ifAndOnlyIf(Filter<E> otherFIlter) {
		return data -> apply(data) == otherFIlter.apply(data);
	}

}
