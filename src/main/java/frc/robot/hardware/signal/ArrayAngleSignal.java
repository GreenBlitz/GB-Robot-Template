package frc.robot.hardware.signal;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.InputSignal;
import frc.utils.AngleUnit;
import frc.utils.TimedValue;
import frc.utils.math.ToleranceMath;
import org.littletonrobotics.junction.LogTable;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

public abstract class ArrayAngleSignal implements InputSignal<Rotation2d> {

	private final String name;
	protected final AngleUnit angleUnit;
	private final List<TimedValue<Rotation2d>> timedValues;

	public ArrayAngleSignal(String name, AngleUnit angleUnit) {
		this.name = name;
		this.angleUnit = angleUnit;
		this.timedValues = new ArrayList<>();
	}

	private TimedValue<Rotation2d> getLatestTimedValue() {
		if (timedValues.isEmpty()) {
			return new TimedValue<>(new Rotation2d(), 0);
		}
		return timedValues.get(timedValues.size() - 1);
	}

	@Override
	public String getName() {
		return name;
	}

	@Override
	public int getNumberOfValues() {
		return timedValues.size();
	}

	@Override
	public Rotation2d getLatestValue() {
		return getLatestTimedValue().getValue();
	}

	@Override
	public Rotation2d[] asArray() {
		return timedValues.stream().map(TimedValue::getValue).toArray(Rotation2d[]::new);
	}

	public Iterator<Rotation2d> iterator() {
		return timedValues.stream().map(TimedValue::getValue).iterator();
	}

	@Override
	public double getTimestamp() {
		return getLatestTimedValue().getTimestamp();
	}

	@Override
	public double[] getTimestamps() {
		return timedValues.stream().mapToDouble(TimedValue::getTimestamp).toArray();
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
		updateValues(timedValues);
		table.put(name, asArray());
	}

	@Override
	public void fromLog(LogTable table) {}

	@Override
	public Rotation2d getAndUpdateValue() {
		updateValues(timedValues);
		return getLatestValue();
	}

	protected abstract void updateValues(List<TimedValue<Rotation2d>> timedValues);

}
