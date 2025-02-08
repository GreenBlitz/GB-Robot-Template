package frc.utils.math;

import java.util.function.Function;

public class DataMath {

	public static <T extends Iterable<E>, E> double calculateStandardDeviations(T values, Function<E, Double> getValue) {
		int length = 0;
		double total = 0;
		for (E value : values) {
			length++;
			total += getValue.apply(value);
		}
		double mean = total / length;
		double deviationsTotal = 0;
		for (E value : values) {
			deviationsTotal += Math.pow(getValue.apply(value) - mean, 2);
		}
		return Math.sqrt(deviationsTotal / length);
	}

}
