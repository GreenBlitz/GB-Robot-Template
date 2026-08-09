package frc.robot.hardware.signal;

import frc.robot.hardware.interfaces.InputSignal;
import frc.utils.TimedValue;
import frc.utils.math.ToleranceMath;
import frc.utils.time.TimeUtil;
import org.littletonrobotics.junction.LogTable;
import java.util.ArrayList;
import java.util.List;

public abstract class DoubleSignal implements InputSignal<Double> {

	private final String name;
	private final TimedValue<Double> timedValue;
	private final List<Double> valueBuffer = new ArrayList<>();
	private final List<Double> timestampBuffer = new ArrayList<>();
	private double lastClearTime = 0.0;

	public DoubleSignal(String name) {
		this.name = name;
		this.timedValue = new TimedValue<>(0.0, 0);
	}

	@Override
	public String getName() {
		return name;
	}

	@Override
	public TimedValue<Double> getLatestTimedValue() {
		return timedValue;
	}

	@Override
	public Double getLatestValue() {
		return timedValue.getValue();
	}

	private synchronized void recordSample() {
		double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
		if (now - lastClearTime > 0.015) {
			valueBuffer.clear();
			timestampBuffer.clear();
			lastClearTime = now;
		}
		valueBuffer.add(timedValue.getValue());
		timestampBuffer.add(timedValue.getTimestamp());
	}

	@Override
	public synchronized Double[] asArray() {
		if (valueBuffer.isEmpty()) {
			return new Double[] {timedValue.getValue()};
		}
		return valueBuffer.toArray(new Double[0]);
	}

	@Override
	public double getTimestamp() {
		return timedValue.getTimestamp();
	}

	@Override
	public synchronized double[] getTimestamps() {
		if (timestampBuffer.isEmpty()) {
			return new double[] {timedValue.getTimestamp()};
		}
		double[] array = new double[timestampBuffer.size()];
		for (int i = 0; i < array.length; i++) {
			array[i] = timestampBuffer.get(i);
		}
		return array;
	}

	@Override
	public boolean isNear(Double value, Double tolerance) {
		return ToleranceMath.isNear(value, getLatestValue(), tolerance);
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
		recordSample();
		table.put(name, timedValue.getValue());
	}

	@Override
	public void fromLog(LogTable table) {
		timedValue.setValue(table.get(name, 0.0));
		timedValue.setTimestamp(TimeUtil.getCurrentTimeSeconds());
		recordSample();
	}

	public Double getAndUpdateValue() {
		updateValue(timedValue);
		recordSample();
		return timedValue.getValue();
	}

	protected abstract void updateValue(TimedValue<Double> timedValue);

}
