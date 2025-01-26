package frc.utils.linearfilters;

import edu.wpi.first.math.Num;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.filter.LinearFilter;
import org.ejml.simple.SimpleMatrix;
import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.function.Supplier;
import java.util.stream.IntStream;

public class PeriodicNDimlLinearFilter<T extends Num> implements IPeriodicLinearFilter {

	private final Supplier<? extends List<Vector<T>>> updateValues;
	private final List<LinearFilter> linearFilters;
	private final String name;
	private final T size;

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
		for (int i = 0; i < updateValues.get().size(); i++) {
			Logger.recordOutput(logPath + "input" + i, updateValues.get().get(i));
		}
		Logger.recordOutput(logPath + "output", getAsColumnVector());
	}

	@Override
	public void update() {
		for (Vector<T> data : updateValues.get()) {
			for (int i = 0; i < size.getNum(); i++) {
				linearFilters.get(i).calculate(data.get(0, i));
			}
		}
	}

	public Vector<T> getAsColumnVector() {
		return new Vector<>(new SimpleMatrix(new double[][] {toArray()}));
	}

	private double[] toArray() {
		return IntStream.range(0, size.getNum()).mapToDouble((i) -> linearFilters.get(i).lastValue()).toArray();
	}

}
