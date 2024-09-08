package frc.robot.hardware.signal;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import java.util.ArrayList;
import java.util.Arrays;

public abstract class InputSignal<T> implements LoggableInputs {

	private ArrayList<T> values;

	protected InputSignal(T defaultValue) {
		this.values = new ArrayList<>();
		values.add(defaultValue);
	}

	protected void setNewValues(T... newValues) {
		values = new ArrayList<>(Arrays.asList(newValues));
	}

	public ArrayList<T> asArray() {
		return values;
	}

	public T getLatestValue() {
		return values.get(values.size() - 1);
	}

	@Override
	public void fromLog(LogTable table) {}

}

