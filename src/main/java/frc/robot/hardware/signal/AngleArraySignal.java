package frc.robot.hardware.signal;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.utils.AngleUnit;
import org.littletonrobotics.junction.LogTable;

import java.util.Arrays;

public abstract class AngleArraySignal implements InputSignal<Rotation2d> {

	private final String name;
	private final AngleUnit angleUnit;
	private Rotation2d[] values;
	private double[] timestamps;

	public AngleArraySignal(String name, AngleUnit angleUnit) {
		this.name = name;
		this.angleUnit = angleUnit;
		this.values = new Rotation2d[] {new Rotation2d()};
	}

	@Override
	public Rotation2d getLatestValue() {
		return values[values.length - 1];
	}

	@Override
	public Rotation2d[] asArray() {
		return values;
	}

	@Override
	public double getTimestamp() {
		return timestamps[timestamps.length - 1];
	}

	@Override
	public double[] getTimestamps() {
		return timestamps;
	}

	@Override
	public void toLog(LogTable table) {
		TimedValue<Double>[] timedValues = getNewValues();
		values = Arrays.stream(timedValues).map(timedValue -> angleUnit.toAngle(timedValue.value())).toArray(Rotation2d[]::new);
		timestamps = Arrays.stream(timedValues).mapToDouble(TimedValue::timestamp).toArray();
		table.put(name, values);
	}

	@Override
	public void fromLog(LogTable table) {
		values = table.get(name, new Rotation2d[] {new Rotation2d()});
	}

	protected abstract TimedValue<Double>[] getNewValues();

}
