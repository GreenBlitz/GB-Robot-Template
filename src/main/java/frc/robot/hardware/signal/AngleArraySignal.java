package frc.robot.hardware.signal;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.InputSignal;
import frc.utils.AngleUnit;
import frc.utils.TimedValue;
import frc.utils.math.ToleranceMath;
import org.littletonrobotics.junction.LogTable;

public abstract class AngleArraySignal implements InputSignal<Rotation2d> {

	private final String name;
	protected final AngleUnit angleUnit;
	private TimedValue<Rotation2d>[] timedValues;

	public AngleArraySignal(String name, AngleUnit angleUnit) {
		this.name = name;
		this.angleUnit = angleUnit;
		this.timedValues = new TimedValue[] {new TimedValue<>(new Rotation2d(), 0.0)};
	}

	@Override
	public String getName() {
		return name;
	}

	@Override
	public Rotation2d getLatestValue() {
		if (timedValues.length == 0) {
			return new Rotation2d();
		}
		return timedValues[timedValues.length - 1].getValue();
	}

	@Override
	public Rotation2d[] asArray() {
		Rotation2d[] values = new Rotation2d[timedValues.length];
		for (int i = 0; i < timedValues.length; i++) {
			values[i] = timedValues[i].getValue();
		}
		return values;
	}

	@Override
	public double getTimestamp() {
		return timedValues[timedValues.length - 1].getTimestamp();
	}

	@Override
	public double[] getTimestamps() {
		double[] timestamps = new double[timedValues.length];
		for (int i = 0; i < timedValues.length; i++) {
			timestamps[i] = timedValues[i].getTimestamp();
		}
		return timestamps;
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
		timedValues = updateValues(timedValues);
		table.put(name, asArray());
	}

	@Override
	public void fromLog(LogTable table) {}

	public Rotation2d getAndUpdateValue() {
		update();
		return getLatestValue();
	}

	@Override
	public void update() {
		timedValues = updateValues(timedValues);
	}

	protected abstract TimedValue<Rotation2d>[] updateValues(TimedValue<Rotation2d>[] timedValues);

}
