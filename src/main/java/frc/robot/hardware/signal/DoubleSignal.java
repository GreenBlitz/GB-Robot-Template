package frc.robot.hardware.signal;

import org.littletonrobotics.junction.LogTable;

public abstract class DoubleSignal implements InputSignal<Double> {

	private final String name;
	private double value;

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
	public void toLog(LogTable table) {
		value = getNewValue();
		table.put(name, value);
	}

	@Override
	public void fromLog(LogTable table) {
		value = table.get(name, 0);
	}

	protected abstract Double getNewValue();

}
