package frc.utils;

import java.util.List;
import java.util.Optional;
import java.util.stream.IntStream;

public final class GBMeth {

	/**
	 * Returns the <a herf="https://en.wikipedia.org/wiki/Generalized_mean">generalized weighted mean</a> of the given data with the given
	 * weights. When infinity or 0 is given as a rank, the limit is outputted.
	 *
	 * @param rank:    the rank of the generalized mean.
	 * @param data:    the data to take a mean on.
	 * @param weights: the weights of the data. Precondition: data.size = weights.size;
	 * @return the generalized weighted mean. Returns `Double.NaN` if has zero data.
	 */
	public static double mean(Double rank, List<Double> data, List<Double> weights) {
		int size = Math.min(data.size(), weights.size());
		// insures all the calls for Optional.get are safe
		if (size == 0) {
			return Double.NaN;
		}
		if (rank == 0) {
			return geometricMean(data, weights, size);
		} else if (rank == Double.POSITIVE_INFINITY) {
			return data.stream().max(Double::compareTo).get();
		} else if (rank == Double.NEGATIVE_INFINITY) {
			return data.stream().min(Double::compareTo).get();
		} else if (rank.isNaN()) {
			return Double.NaN;
		}
		double output = IntStream.range(0, size).mapToDouble(i -> Math.pow(data.get(i), rank) * weights.get(i)).sum();
		return Math.pow(output / (sumList(weights)).get(), 1 / rank);
	}

	public static double mean(Double rank, List<Double> data) {
		return mean(rank, data, data.stream().map((x) -> 1D).toList());
	}

	public static double mean(List<Double> data, List<Double> weights) {
		return mean(1D, data, weights);
	}

	public static double mean(List<Double> data) {
		return mean(1D, data);
	}

	private static double geometricMean(List<Double> data, List<Double> weights, int size) {
		double output = 1;
		for (int i = 0; i < size; i++) {
			output *= Math.pow(data.get(i), weights.get(i));
		}
		Optional<Double> weightsSum = sumList(weights);
		return weightsSum.isPresent() ? Math.pow(output, 1 / weightsSum.get()) : Double.NaN;
	}

	public static Optional<Double> sumList(List<Double> weights) {
		return weights.stream().reduce(Double::sum);
	}

}
