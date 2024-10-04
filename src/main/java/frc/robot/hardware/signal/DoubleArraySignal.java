package frc.robot.hardware.signal;


import org.littletonrobotics.junction.LogTable;

import java.util.stream.DoubleStream;

public abstract class DoubleArraySignal implements InputSignal<Double> {

	private final String name;
	private double[] values;

	public DoubleArraySignal(String name) {
		this.name = name;
		this.values = new double[] {0};
	}

	@Override
	public Double getLatestValue() {
		return values[values.length - 1];
	}

	@Override
	public Double[] asArray() {
		return DoubleStream.of(values).boxed().toArray(Double[]::new);
	}

	@Override
	public void toLog(LogTable table) {
		values = getNewValues();
		table.put(name, values);
	}

	@Override
	public void fromLog(LogTable table) {
		values = table.get(name, new double[] {0});
	}

	protected abstract double[] getNewValues();

}
