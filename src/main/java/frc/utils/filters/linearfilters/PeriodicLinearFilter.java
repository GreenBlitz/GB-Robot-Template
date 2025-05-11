package frc.utils.filters.linearfilters;

import edu.wpi.first.math.filter.LinearFilter;
import frc.utils.filters.IPeriodicFilter;
import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.function.Supplier;

/**
 * A linear filter that support <code>LinearFiltersManager</code>, and that can be handled periodically by it.
 */
public class PeriodicLinearFilter implements IPeriodicFilter<Double> {

	private final LinearFilter filter;
	private final String name;
	private final Supplier<? extends List<Double>> updateValues;

	public PeriodicLinearFilter(LinearFilter filter, String name, Supplier<? extends List<Double>> updateValues) {
		this.filter = filter;
		this.name = name;
		this.updateValues = updateValues;
	}

	public Double get() {
		return filter.lastValue();
	}

	public void hardReset() {
		filter.reset();
	}

	public void log(String parentLogPath) {
		String logPath = parentLogPath + name + "/";
		Logger.recordOutput(logPath + "output", get());
	}

	public void update() {
		for (Double input : updateValues.get()) {
			filter.calculate(input);
		}
	}

}
