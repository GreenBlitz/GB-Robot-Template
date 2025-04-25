package frc.utils;


public interface Filter<T> {

	static <T> Filter<T> nonFilteringFilter() {
		return data -> true;
	}

	static <T> Filter<T> alwaysFilteringFilter() {
		return data -> false;
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

	default <E extends T> Filter<E> nand(Filter<E> otherFilter) {
		return data -> and(otherFilter).not().apply(data);
	}

	default <E extends T> Filter<E> nor(Filter<E> otherFilter) {
		return data -> or(otherFilter).not().apply(data);
	}

	default <E extends T> Filter<E> xnor(Filter<E> otherFilter) {
		return data -> xor(otherFilter).not().apply(data);
	}

	default <E extends T> Filter<E> implies(Filter<E> otherFilter) {
		return data -> !apply(data) || otherFilter.apply(data);
	}

	static <T> Filter<T> orAll(Iterable<Filter<T>> otherFilers) {
		Filter<T> output = alwaysFilteringFilter();
		for (Filter<T> filter : otherFilers) {
			output = output.or(filter);
		}
		return output;
	}

	static <T> Filter<T> andAll(Iterable<Filter<T>> otherFilers) {
		Filter<T> output = nonFilteringFilter();
		for (Filter<T> filter : otherFilers) {
			output = output.and(filter);
		}
		return output;
	}

}
