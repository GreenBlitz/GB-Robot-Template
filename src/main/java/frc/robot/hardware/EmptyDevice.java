package frc.robot.hardware;

import frc.robot.hardware.signal.InputSignal;
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
	public void fetchSignals(InputSignal... signals) {}

}
