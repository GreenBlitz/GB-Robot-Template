package frc.robot.hardware.signal;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public abstract class SignalInput implements LoggableInputs {

	private double[] latestValues = {};

	public double[] asArray() {
		return latestValues;
	}

	public double getLatestValue() {
		return latestValues[latestValues.length - 1];
	}

	@Override
	public void fromLog(LogTable table) {}

}
