package frc.robot.hardware.signal;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.DoubleFunction;


public abstract class InputSignal<T> implements LoggableInputs {

	public abstract static class DoubleSignal extends InputSignal<Double> {

		protected DoubleSignal() {
			super(0.0);
		}

	}

	public abstract static class AngleSignal extends InputSignal<Rotation2d> {

		private final DoubleFunction<Rotation2d> toAngle;

		protected AngleSignal(DoubleFunction<Rotation2d> toAngle) {
			super(new Rotation2d());
			this.toAngle = toAngle;
		}

		protected void setNewValues(double[] newValues) {
			setNewValues(Arrays.stream(newValues).mapToObj(toAngle).toArray(Rotation2d[]::new));
		}

		protected void setNewValues(double newValue) {
			setNewValues(toAngle.apply(newValue));
		}

	}

	private ArrayList<T> values;

	protected InputSignal(T defaultValue) {
		this.values = new ArrayList<>();
		values.add(defaultValue);
	}

	protected void setNewValues(T[] newValues) {
		values = new ArrayList<>(Arrays.asList(newValues));
	}

	protected void setNewValues(T newValue) {
		ArrayList<T> arrayList = new ArrayList<>();
		arrayList.add(newValue);
		values = arrayList;
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

