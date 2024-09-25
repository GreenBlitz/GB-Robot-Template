package frc.robot.hardware.signal;

import org.littletonrobotics.junction.LogTable;

public class EmptyDoubleSignal implements InputSignal<Double> {

	private final String name;
    private double defaultValue;

    public EmptyDoubleSignal(String name, Double defaultValue) {
        this.name = name;
        this.defaultValue = defaultValue;
	}

	@Override
	public Double getLatestValue() {
		return defaultValue;
	}

	@Override
	public Double[] asArray() {
		return new Double[] {defaultValue};
	}

	@Override
	public double getTimestamp() {
		return 0;
	}

	@Override
	public double[] getTimestamps() {
		return new double[0];
	}

	@Override
	public void toLog(LogTable table) {
        table.put(name, defaultValue);
    }

	@Override
	public void fromLog(LogTable table) {
        defaultValue = table.get(name, defaultValue);
    }

}
