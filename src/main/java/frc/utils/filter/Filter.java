package frc.utils.filter;

public interface Filter {

	boolean isPassFilter();

	default Filter not() {
		return () -> !isPassFilter();
	}

	default Filter and(Filter other) {
		return () -> isPassFilter() && other.isPassFilter();
	}

	default Filter or(Filter other) {
		return () -> isPassFilter() || other.isPassFilter();
	}

	default Filter xor(Filter other) {
		return () -> isPassFilter() ^ other.isPassFilter();
	}

	default Filter nand(Filter other) {
		return and(other).not();
	}

	default Filter nor(Filter other) {
		return or(other).not();
	}

	default Filter xnor(Filter other) {
		return xor(other).not();
	}

	default Filter implies(Filter other) {
		return not().or(other);
	}

	static Filter nonFilteringFilter() {
		return () -> true;
	}

	static Filter alwaysFilteringFilter() {
		return () -> false;
	}

	static Filter orAll(Filter... filters) {
		Filter output = alwaysFilteringFilter();
		for (Filter filter : filters) {
			output = output.or(filter);
		}
		return output;
	}

	static Filter andAll(Filter... filters) {
		Filter output = nonFilteringFilter();
		for (Filter filter : filters) {
			output = output.and(filter);
		}
		return output;
	}

}
