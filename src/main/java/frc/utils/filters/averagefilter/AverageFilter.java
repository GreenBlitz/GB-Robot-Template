package frc.utils.filters.averagefilter;

import edu.wpi.first.math.Num;
import edu.wpi.first.math.Vector;
import frc.utils.GBMeth;
import frc.utils.filters.linearfilters.IPeriodicFilter;
import frc.utils.pose.PoseUtils;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

/**
 * This class is a filter that uses a weighted generalized average to take finite amount of numbers and transform them to a single number.
 * Independent of previous observation.
 */
public class AverageFilter<T extends Num> implements IPeriodicFilter<Double> {

	private final String logPathAddition;
	private final double rank;
	private final Supplier<Vector<T>> dataSupplier;
	private final T size;
	private final Vector<T> weights;

	private Double lastOutput;

	public AverageFilter(String logPathAddition, double rank, Supplier<Vector<T>> dataSupplier, T size, Vector<T> weights) {
		this.logPathAddition = logPathAddition;
		this.rank = rank;
		this.size = size;
		this.dataSupplier = dataSupplier;
		this.weights = weights;

		update();
	}

	// Independent of previous observations: empty implementation
	@Override
	public void hardReset() {}

	@Override
	public void log(String parentLogPath) {
		String logPath = parentLogPath + logPathAddition;
		Logger.recordOutput(logPath + "output", get());
		Logger.recordOutput(logPath + "rank", rank);
		Logger.recordOutput(logPath + "size", size.getNum());
		Logger.recordOutput(logPath + "weights", PoseUtils.vectorToArray(weights, size));
	}

	@Override
	public void update() {
		lastOutput = GBMeth.mean(rank, PoseUtils.vectorToList(dataSupplier.get(), size), PoseUtils.vectorToList(weights, size));
	}

	@Override
	public Double get() {
		return lastOutput;
	}

}
