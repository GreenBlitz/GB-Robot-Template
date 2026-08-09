package frc.robot.hardware.signal;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.InputSignal;
import frc.utils.AngleUnit;
import frc.utils.TimedValue;
import frc.utils.math.ToleranceMath;
import frc.utils.time.TimeUtil;
import org.littletonrobotics.junction.LogTable;
import java.util.ArrayList;
import java.util.List;

public abstract class AngleSignal implements InputSignal<Rotation2d> {

	private final String name;
	protected final AngleUnit angleUnit;
	private final TimedValue<Rotation2d> timedValue;
	private final List<Rotation2d> valueBuffer = new ArrayList<>();
	private final List<Double> timestampBuffer = new ArrayList<>();
	private double lastClearTime = 0.0;

	public AngleSignal(String name, AngleUnit angleUnit) {
		this.name = name;
		this.angleUnit = angleUnit;
		this.timedValue = new TimedValue<>(new Rotation2d(), 0);
	}

	@Override
	public String getName() {
		return name;
	}

	@Override
	public TimedValue<Rotation2d> getLatestTimedValue() {
		return timedValue;
	}

	@Override
	public Rotation2d getLatestValue() {
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
	public synchronized Rotation2d[] asArray() {
		if (valueBuffer.isEmpty()) {
			return new Rotation2d[] {timedValue.getValue()};
		}
		return valueBuffer.toArray(new Rotation2d[0]);
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
	public boolean isNear(Rotation2d value, Rotation2d tolerance) {
		return ToleranceMath.isNear(value.getRotations(), getLatestValue().getRotations(), tolerance.getRotations());
	}

	@Override
	public boolean isFurther(Rotation2d value, Rotation2d tolerance) {
		return !isNear(value, tolerance);
	}

	@Override
	public boolean isGreater(Rotation2d value) {
		return getLatestValue().getRotations() > value.getRotations();
	}

	@Override
	public boolean isLess(Rotation2d value) {
		return getLatestValue().getRotations() < value.getRotations();
	}

	@Override
	public void toLog(LogTable table) {
		updateValue(timedValue);
		recordSample();
		table.put(name, timedValue.getValue());
	}

	@Override
	public void fromLog(LogTable table) {
		timedValue.setValue(table.get(name, new Rotation2d()));
		timedValue.setTimestamp(TimeUtil.getCurrentTimeSeconds());
		recordSample();
	}

	public Rotation2d getAndUpdateValue() {
		updateValue(timedValue);
		recordSample();
		return timedValue.getValue();
	}

	protected abstract void updateValue(TimedValue<Rotation2d> timedValue);

}
