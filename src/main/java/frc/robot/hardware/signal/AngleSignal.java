package frc.robot.hardware.signal;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.utils.AngleUnit;
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
	public void toLog(LogTable table) {
		TimedValue<Double> timedValue = getNewValue();
		value = angleUnit.toAngle(timedValue.value());
		timestamp = timedValue.timestamp();
		table.put(name, value);
	}

	@Override
	public void fromLog(LogTable table) {
		value = table.get(name, new Rotation2d());
	}

	protected abstract TimedValue<Double> getNewValue();

}
