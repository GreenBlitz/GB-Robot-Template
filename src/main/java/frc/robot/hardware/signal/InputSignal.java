package frc.robot.hardware.signal;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public abstract class InputSignal implements LoggableInputs {

	private double[] latestValues = {};

	protected void setNewValues(double[] newValues) {
		latestValues = newValues;
	}

	protected void setNewValues(double newValue) {
		latestValues = new double[] {newValue};
	}

	public double[] asArray() {
		return latestValues;
	}

	public double getLatestValue() {
		return latestValues[latestValues.length - 1];
	}

	@Override
	public void fromLog(LogTable table) {}

}
