package frc.robot.hardware.signal;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.utils.AngleUnit;
import frc.utils.TimedValue;
import frc.utils.time.TimeUtil;
import org.littletonrobotics.junction.LogTable;

public class MultiSampleAngleSignal extends AngleSignal {

	private Rotation2d[] values;
	private double[] timestamps;

	public MultiSampleAngleSignal(String name, AngleUnit angleUnit) {
		super(name, angleUnit);
		this.values = new Rotation2d[] {new Rotation2d()};
		this.timestamps = new double[] {0};
	}

	public void setValues(Rotation2d[] values, double[] timestamps) {
		if (values.length == 0 || values.length != timestamps.length) {
			return;
		}
		this.values = values;
		this.timestamps = timestamps;
		getLatestTimedValue().setValue(values[values.length - 1]);
		getLatestTimedValue().setTimestamp(timestamps[timestamps.length - 1]);
	}

	public void setLatestValue(Rotation2d value, double timestamp) {
		setValues(new Rotation2d[] {value}, new double[] {timestamp});
	}

	@Override
	public Rotation2d[] asArray() {
		return values;
	}

	@Override
	public double[] getTimestamps() {
		return timestamps;
	}

	@Override
	protected void updateValue(TimedValue<Rotation2d> timedValue) {}

	@Override
	public void toLog(LogTable table) {
		table.put(getName(), values);
		table.put(getName() + "Timestamps", timestamps);
	}

	@Override
	public void fromLog(LogTable table) {
		Rotation2d[] loggedValues = table.get(getName(), new Rotation2d[] {new Rotation2d()});
		double[] loggedTimestamps = table.get(getName() + "Timestamps", new double[] {TimeUtil.getCurrentTimeSeconds()});
		int validLength = Math.min(loggedValues.length, loggedTimestamps.length);
		if (validLength > 0) {
			if (validLength != loggedValues.length || validLength != loggedTimestamps.length) {
				Rotation2d[] trimmedValues = new Rotation2d[validLength];
				double[] trimmedTimestamps = new double[validLength];
				System.arraycopy(loggedValues, 0, trimmedValues, 0, validLength);
				System.arraycopy(loggedTimestamps, 0, trimmedTimestamps, 0, validLength);
				setValues(trimmedValues, trimmedTimestamps);
			} else {
				setValues(loggedValues, loggedTimestamps);
			}
		}
	}

}
