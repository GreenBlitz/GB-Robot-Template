package frc.utils.filters.linearfilters;

import edu.wpi.first.math.Num;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.filter.LinearFilter;
import frc.utils.filters.IPeriodicFilter;
import org.ejml.simple.SimpleMatrix;
import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.function.Supplier;
import java.util.stream.IntStream;

public class PeriodicNDimlLinearFilter<T extends Num> implements IPeriodicFilter<Vector<T>> {

	private final Supplier<? extends List<Vector<T>>> updateValues;
	private final List<LinearFilter> linearFilters;
	private final String name;
	private final T size;

	/**
	 * creates a n-dim periodic linear filter
	 *
	 * @param updateValues: supplies all the values (as n-dim vectors) in the current periodic circle
	 * @param filters:      n-sized list, that injects the filters for each value of the given vector
	 * @param name:         the name of the periodic filter
	 * @param sizeInstance: an instance of wpilib's Num (represent the size of the vectors)
	 */
	public PeriodicNDimlLinearFilter(Supplier<List<Vector<T>>> updateValues, List<LinearFilter> filters, String name, T sizeInstance) {
		this.updateValues = updateValues;
		this.linearFilters = filters;
		this.name = name;
		this.size = sizeInstance;
		Logger.recordOutput("actualSize", updateValues.get().get(0).unit());
	}

	@Override
	public void hardReset() {
		linearFilters.forEach(LinearFilter::reset);
	}

	@Override
	public void log(String parentLogPath) {
		String logPath = parentLogPath + name + "/";
		for (int i = 0; i < size.getNum(); i++) {
			Logger.recordOutput(logPath + "/" + i, get().get(0, i));
		}
	}

	@Override
	public void update() {
		for (Vector<T> data : updateValues.get()) {
			for (int i = 0; i < size.getNum(); i++) {
				linearFilters.get(i).calculate(data.get(0, i));
			}
		}
	}

	public Vector<T> get() {
		return new Vector<>(new SimpleMatrix(new double[][] {toArray()}));
	}

	private double[] toArray() {
		return IntStream.range(0, size.getNum()).mapToDouble((i) -> linearFilters.get(i).lastValue()).toArray();
	}

}
