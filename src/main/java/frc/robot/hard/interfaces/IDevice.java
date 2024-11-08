package frc.robot.hard.interfaces;

public interface IDevice {

	boolean isConnected();

	void updateInputs(InputSignal<?>... inputSignals);

}
