package frc.robot.hardware.newsignal;

import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface InputSignal<T> extends LoggableInputs {

    T getLatestValue();

    T[] asArray();

}
