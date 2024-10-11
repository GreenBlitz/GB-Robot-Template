package frc.robot.hardware.signal;

import org.littletonrobotics.junction.LogTable;

public abstract class DoubleSignal implements InputSignal<Double> {

	private final String name;
	private double value;
	private double timestamp;

	public DoubleSignal(String name) {
		this.name = name;
		this.value = 0;
	}

	@Override
	public Double getLatestValue() {
		return value;
	}

	@Override
	public Double[] asArray() {
		return new Double[] {value};
	}

	@Override
	public double getTimestamp() {
		return timestamp;
	}

	@Override
	public double[] getTimestamps() {
		return new double[] {timestamp};
	}

	@Override
	public void toLog(LogTable table) {
		TimedValue<Double> timedValue = getNewValue();
		value = timedValue.value();
		timestamp = timedValue.timestamp();
		table.put(name, value);
	}

	@Override
	public void fromLog(LogTable table) {
		value = table.get(name, 0);
	}

	protected abstract TimedValue<Double> getNewValue();

}
