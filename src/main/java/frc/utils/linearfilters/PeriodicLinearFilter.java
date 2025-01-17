package frc.utils.linearfilters;

import edu.wpi.first.math.filter.LinearFilter;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

/**
 * A linear filter that support <code>LinearFiltersManager</code>, and that can be handled periodically by it.
 */
public class PeriodicLinearFilter implements IPeriodicLinearFilter {

	private final LinearFilter filter;
	private final String name;
	private final Supplier<Double> updateValue;

	public PeriodicLinearFilter(LinearFilter filter, String name, Supplier<Double> updateValue) {
		this.filter = filter;
		this.name = name;
		this.updateValue = updateValue;
	}

	public double getOutput() {
		return filter.lastValue();
	}

	public void hardReset() {
		filter.reset();
	}

	protected void log(String parentLogPath) {
		String logPath = parentLogPath + name + "/";
		Logger.recordOutput(logPath + "input", updateValue.get());
		Logger.recordOutput(logPath + "output", getOutput());
	}

	protected void update() {
		filter.calculate(updateValue.get());
	}

}
