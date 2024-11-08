package frc.robot.hardware.interfaces;

public interface IDevice {

	boolean isConnected();

	void updateInputs(InputSignal<?>... inputSignals);

}
