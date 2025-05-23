package frc.robot.hardware.signal;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.InputSignal;
import frc.utils.AngleUnit;
import frc.utils.TimedValue;
import frc.utils.math.ToleranceMath;
import org.littletonrobotics.junction.LogTable;

public abstract class AngleSignal implements InputSignal<Rotation2d> {

	private final String name;
	protected final AngleUnit angleUnit;
	private final TimedValue<Rotation2d> timedValue;

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
	public Rotation2d getLatestValue() {
		return timedValue.getValue();
	}

	@Override
	public Rotation2d[] asArray() {
		return new Rotation2d[] {timedValue.getValue()};
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
		table.put(name, timedValue.getValue());
	}

	@Override
	public void fromLog(LogTable table) {
		timedValue.setValue(table.get(name, new Rotation2d()));
	}

	public Rotation2d getAndUpdateValue() {
		updateValue(timedValue);
		return timedValue.getValue();
	}

	protected abstract void updateValue(TimedValue<Rotation2d> timedValue);

}
