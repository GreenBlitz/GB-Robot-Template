package frc.robot.hardware.signal;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.LogTable;

public class EmptyAngleSignal implements InputSignal<Rotation2d> {

	private final String name;
	private Rotation2d defaultValue;

	public EmptyAngleSignal(String name, Rotation2d defaultValue) {
		this.name = name;
		this.defaultValue = defaultValue;
	}

	@Override
	public Rotation2d getLatestValue() {
		return defaultValue;
	}

	@Override
	public Rotation2d[] asArray() {
		return new Rotation2d[] {defaultValue};
	}

	@Override
	public double getTimestamp() {
		return 0;
	}

	@Override
	public double[] getTimestamps() {
		return new double[0];
	}

	@Override
	public void toLog(LogTable table) {
		table.put(name, defaultValue);
	}

	@Override
	public void fromLog(LogTable table) {
		defaultValue = table.get(name, defaultValue);
	}

}
