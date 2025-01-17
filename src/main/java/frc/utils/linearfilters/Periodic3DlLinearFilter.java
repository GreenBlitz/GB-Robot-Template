package frc.utils.linearfilters;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.numbers.N3;
import org.ejml.simple.SimpleMatrix;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

public class Periodic3DlLinearFilter implements IPeriodicLinearFilter {

	private final ArrayList<Supplier<Double>> suppliers;
	private final ArrayList<LinearFilter> linearFilters;
	private final String name;

	public Periodic3DlLinearFilter(
		Supplier<Double> xSupplier,
		Supplier<Double> ySupplier,
		Supplier<Double> zSupplier,
		LinearFilter xFilter,
		LinearFilter yFilter,
		LinearFilter zFilter,
		String name
	) {
		this.suppliers = new ArrayList<>(List.of(xSupplier, ySupplier, zSupplier)); // safe casting
		this.linearFilters = new ArrayList<>(List.of(xFilter, yFilter, zFilter));
		this.name = name;
	}

	@Override
	public void hardReset() {
		linearFilters.forEach(LinearFilter::reset);
	}

	@Override
	public void log(String parentLogPath) {
		String logPath = parentLogPath + name + "/";
		Logger.recordOutput(logPath + "input/" + "x/", suppliers.get(0).get());
		Logger.recordOutput(logPath + "input/" + "y/", suppliers.get(1).get());
		Logger.recordOutput(logPath + "input/" + "z/", suppliers.get(2).get());
		Logger.recordOutput(logPath + "output/" + "x/", linearFilters.get(0).lastValue());
		Logger.recordOutput(logPath + "output/" + "y/", linearFilters.get(0).lastValue());
		Logger.recordOutput(logPath + "output/" + "z/", linearFilters.get(0).lastValue());
	}

	@Override
	public void update() {
		for (int i = 0; i < 3; i++) {
			linearFilters.get(i).calculate(suppliers.get(i).get());
		}
	}

	public Vector<N3> getAsColumnVector() {
		return new Vector<>(new SimpleMatrix(new double[][] {getAsPoseArray()}));
	}

	private double[] getAsPoseArray() {
		return new double[] {linearFilters.get(0).lastValue(), linearFilters.get(1).lastValue(), linearFilters.get(2).lastValue()};
	}

}
