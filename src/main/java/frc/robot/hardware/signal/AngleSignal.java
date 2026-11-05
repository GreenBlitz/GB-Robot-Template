package frc.robot.hardware.signal;

import frc.utils.costomUnits.Angle2d;
import frc.robot.hardware.interfaces.InputSignal;
import frc.utils.AngleUnit;
import frc.utils.TimedValue;
import frc.utils.math.ToleranceMath;
import frc.utils.time.TimeUtil;
import org.littletonrobotics.junction.LogTable;

public abstract class AngleSignal implements InputSignal<Angle2d> {

	private final String name;
	protected final AngleUnit angleUnit;
	private final TimedValue<Angle2d> timedValue;

	public AngleSignal(String name, AngleUnit angleUnit) {
		this.name = name;
		this.angleUnit = angleUnit;
		this.timedValue = new TimedValue<>(new Angle2d(), 0);
	}

	@Override
	public String getName() {
		return name;
	}

	@Override
	public TimedValue<Angle2d> getLatestTimedValue() {
		return timedValue;
	}

	@Override
	public Angle2d getLatestValue() {
		return timedValue.getValue();
	}

	@Override
	public Angle2d[] asArray() {
		return new Angle2d[] {timedValue.getValue()};
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
	public boolean isNear(Angle2d value, Angle2d tolerance) {
		return ToleranceMath.isNear(value.getRotations(), getLatestValue().getRotations(), tolerance.getRotations());
	}

	@Override
	public boolean isFurther(Angle2d value, Angle2d tolerance) {
		return !isNear(value, tolerance);
	}

	@Override
	public boolean isGreater(Angle2d value) {
		return getLatestValue().getRotations() > value.getRotations();
	}

	@Override
	public boolean isLess(Angle2d value) {
		return getLatestValue().getRotations() < value.getRotations();
	}

	@Override
	public void toLog(LogTable table) {
		updateValue(timedValue);
		table.put(name, timedValue.getValue());
	}

	@Override
	public void fromLog(LogTable table) {
		timedValue.setValue(table.get(name, new Angle2d()));
		timedValue.setTimestamp(TimeUtil.getCurrentTimeSeconds());
	}

	public Angle2d getAndUpdateValue() {
		updateValue(timedValue);
		return timedValue.getValue();
	}

	protected abstract void updateValue(TimedValue<Angle2d> timedValue);

}
