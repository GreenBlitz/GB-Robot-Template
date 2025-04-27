package frc.utils;

public class TimedValue<T> {

	private T value;
	private double timestamp;

	public TimedValue(T value, double timestamp) {
		this.value = value;
		this.timestamp = timestamp;
	}

	public T getValue() {
		return value;
	}

	public void setValue(T value) {
		this.value = value;
	}

	public double getTimestamp() {
		return timestamp;
	}

	public void setTimestamp(double timestamp) {
		this.timestamp = timestamp;
	}

}
