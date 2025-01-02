package frc.robot.hardware.empties;

import frc.robot.hardware.ConnectedInputAutoLogged;
import frc.robot.hardware.interfaces.IDevice;
import frc.robot.hardware.interfaces.InputSignal;
import org.littletonrobotics.junction.Logger;

public abstract class EmptyDevice implements IDevice {

	private final String logPath;
	private final ConnectedInputAutoLogged connectedInput;

	public EmptyDevice(String logPath) {
		this.logPath = logPath;
		this.connectedInput = new ConnectedInputAutoLogged();
	}

	@Override
	public boolean isConnected() {
		Logger.processInputs(logPath, connectedInput);
		return connectedInput.connected;
	}

	@Override
	public void updateInputs(InputSignal<?>... inputSignals) {}

}
