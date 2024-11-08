package frc.robot.hard.empties;

import frc.robot.hard.ConnectedInputAutoLogged;
import frc.robot.hard.interfaces.IDevice;
import frc.robot.hard.interfaces.InputSignal;
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
