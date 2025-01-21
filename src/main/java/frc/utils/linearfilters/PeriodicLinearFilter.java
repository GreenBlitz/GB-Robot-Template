package frc.utils.linearfilters;

import edu.wpi.first.math.filter.LinearFilter;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.function.Supplier;

/**
 * A linear filter that support <code>LinearFiltersManager</code>, and that can be handled periodically by it.
 */
public class PeriodicLinearFilter implements IPeriodicLinearFilter {

	private final LinearFilter filter;
	private final String name;
	private final Supplier<ArrayList<Double>> updateValues;

	public PeriodicLinearFilter(LinearFilter filter, String name, Supplier<ArrayList<Double>> updateValues) {
		this.filter = filter;
		this.name = name;
		this.updateValues = updateValues;
	}

	public double getOutput() {
		return filter.lastValue();
	}

	public void hardReset() {
		filter.reset();
	}

	public void log(String parentLogPath) {
		String logPath = parentLogPath + name + "/";
		for (int i = 0; i < updateValues.get().size(); i++) {
			Logger.recordOutput(logPath + "input" + i, updateValues.get().get(i));
		}
		Logger.recordOutput(logPath + "output", getOutput());
	}

	public void update() {
		for (Double input : updateValues.get()) {
			filter.calculate(input);
		}
	}

}
