package frc.robot.hardware;

import frc.robot.hardware.signal.InputSignal;

public interface IDevice {

	boolean isConnected();

	void updateInputs(InputSignal<?>... inputSignals);

}
