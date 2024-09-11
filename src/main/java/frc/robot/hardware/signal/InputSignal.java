package frc.robot.hardware.signal;

import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface InputSignal<T> extends LoggableInputs {

	T getLatestValue();

	T[] asArray();

}
