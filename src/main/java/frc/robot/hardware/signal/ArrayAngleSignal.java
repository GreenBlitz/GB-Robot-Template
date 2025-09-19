package frc.robot.hardware.signal;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.InputSignal;
import frc.utils.AngleUnit;
import frc.utils.TimedValue;
import frc.utils.math.ToleranceMath;
import org.littletonrobotics.junction.LogTable;

import java.util.ArrayList;
import java.util.Arrays;

public abstract class ArrayAngleSignal implements InputSignal<Rotation2d> {

	private final String name;
	protected final AngleUnit angleUnit;
	private ArrayList<TimedValue<Rotation2d>> timedValues;

	public ArrayAngleSignal(String name, AngleUnit angleUnit) {
		this.name = name;
		this.angleUnit = angleUnit;
		this.timedValues = new ArrayList<>();
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
		if (timedValues.isEmpty()) {
			return new Rotation2d();
		}
		return timedValues.get(timedValues.size() - 1).getValue();
	}

	@Override
	public Rotation2d[] asArray() {
		Rotation2d[] values = new Rotation2d[timedValues.size()];
		Arrays.setAll(values, index -> timedValues.get(index).getValue());
		return values;
	}

	@Override
	public double getTimestamp() {
		return timedValues.get(timedValues.size() - 1).getTimestamp();
	}

	@Override
	public double[] getTimestamps() {
		double[] timestamps = new double[timedValues.size()];
		Arrays.setAll(timestamps, index -> timedValues.get(index).getTimestamp());
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
		update();
		table.put(name, asArray());
	}

	@Override
	public void fromLog(LogTable table) {}

	@Override
	public Rotation2d getAndUpdateValue() {
		update();
		return getLatestValue();
	}

	public void update() {
		timedValues = updateValues(timedValues);
	}

	protected abstract ArrayList<TimedValue<Rotation2d>> updateValues(ArrayList<TimedValue<Rotation2d>> timedValues);

}
