package frc.robot.hardware.interfaces;

import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface InputSignal<T> extends LoggableInputs {

	String getName();

	T getLatestValue();

	T[] asArray();

	double getTimestamp();

	double[] getTimestamps();

	boolean isNear(T value, T tolerance);

	boolean isFurther(T value, T tolerance);

	boolean isGreater(T value);

	boolean isLess(T value);

}
