package frc.robot.hardware.signal;

import edu.wpi.first.math.MathUtil;
import frc.robot.hardware.interfaces.InputSignal;
import frc.utils.TimedValue;
import org.littletonrobotics.junction.LogTable;

public abstract class DoubleSignal implements InputSignal<Double> {

	private final String name;
	private final TimedValue<Double> timedValue;

	public DoubleSignal(String name) {
		this.name = name;
		this.timedValue = new TimedValue<>(0.0, 0);
	}

	@Override
	public String getName() {
		return name;
	}

	@Override
	public Double getLatestValue() {
		return timedValue.getValue();
	}

	@Override
	public Double[] asArray() {
		return new Double[] {timedValue.getValue()};
	}

	@Override
	public double getTimestamp() {
		return timedValue.getTimestamp();
	}

	@Override
	public double[] getTimestamps() {
		return new double[] {timedValue.getTimestamp()};
	}

	@Override
	public boolean isNear(Double value, Double tolerance) {
		return MathUtil.isNear(value, getLatestValue(), tolerance);
	}

	@Override
	public boolean isFurther(Double value, Double tolerance) {
		return !isNear(value, tolerance);
	}

	@Override
	public boolean isGreater(Double value) {
		return getLatestValue() > value;
	}

	@Override
	public boolean isLess(Double value) {
		return getLatestValue() < value;
	}

	@Override
	public void toLog(LogTable table) {
		updateValue(timedValue);
		table.put(name, timedValue.getValue());
	}

	@Override
	public void fromLog(LogTable table) {
		timedValue.setValue(table.get(name, 0.0));
	}

	public Double getAndUpdateValue() {
		updateValue(timedValue);
		return timedValue.getValue();
	}

	protected abstract void updateValue(TimedValue<Double> timedValue);


}
