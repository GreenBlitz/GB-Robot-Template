package frc.robot.hardware.interfaces;

import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface InputSignal<T> extends LoggableInputs {

    String getName();

    T getLatestValue();

    T[] asArray();

    double getTimestamp();

    double[] getTimestamps();
}
