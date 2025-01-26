package frc.utils.linearfilters;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.numbers.N3;
import frc.robot.poseestimator.Pose2dComponentsValue;
import org.ejml.simple.SimpleMatrix;
import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.function.Supplier;

public class Periodic3DlLinearFilter implements IPeriodicLinearFilter {

	private final Supplier<? extends List<Vector<N3>>> updateValues;
	private final List<LinearFilter> linearFilters;
	private final String name;

	public Periodic3DlLinearFilter(
		Supplier<List<Vector<N3>>> updateValues,
		LinearFilter xFilter,
		LinearFilter yFilter,
		LinearFilter zFilter,
		String name
	) {
		this.updateValues = updateValues;
		this.linearFilters = List.of(xFilter, yFilter, zFilter);
		this.name = name;
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
		for (Vector<N3> data : updateValues.get()) {
			for (int i = 0; i < Pose2dComponentsValue.POSE2D_COMPONENTS_AMOUNT; i++) {
				linearFilters.get(i).calculate(data.get(i));
			}
		}
	}

	public Vector<N3> getAsColumnVector() {
		return new Vector<>(new SimpleMatrix(new double[][] {getAsPoseArray()}));
	}

	private double[] getAsPoseArray() {
		return new double[] {linearFilters.get(0).lastValue(), linearFilters.get(1).lastValue(), linearFilters.get(2).lastValue()};
	}

}
