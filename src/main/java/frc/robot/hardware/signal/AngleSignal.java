package frc.robot.hardware.signal;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.InputSignal;
import frc.utils.AngleUnit;
import frc.utils.TimedValue;
import org.littletonrobotics.junction.LogTable;

public abstract class AngleSignal implements InputSignal<Rotation2d> {

	private final String name;
	private final AngleUnit angleUnit;
	private Rotation2d value;
	private double timestamp;

	public AngleSignal(String name, AngleUnit angleUnit) {
		this.name = name;
		this.angleUnit = angleUnit;
		this.value = new Rotation2d();
	}

	@Override
	public String getName() {
		return name;
	}

	@Override
	public Rotation2d getLatestValue() {
		return value;
	}

	@Override
	public Rotation2d[] asArray() {
		return new Rotation2d[] {value};
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
	public boolean isNear(Rotation2d value, Rotation2d tolerance) {
		return MathUtil.isNear(value.getRotations(), getLatestValue().getRotations(), tolerance.getRotations());
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
		TimedValue<Double> timedValue = getNewValue();
		value = angleUnit.toRotation2d(timedValue.value());
		timestamp = timedValue.timestamp();
		table.put(name, value);
	}

	@Override
	public void fromLog(LogTable table) {
		value = table.get(name, new Rotation2d());
	}

	protected abstract TimedValue<Double> getNewValue();

}
