package frc.robot.hardware.phoenix6;

import com.ctre.phoenix6.hardware.ParentDevice;
import frc.robot.hardware.ConnectedInputAutoLogged;
import frc.robot.hardware.interfaces.IDevice;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.hardware.phoenix6.signal.SignalGetter;
import frc.utils.alerts.Alert;
import frc.utils.alerts.AlertManager;
import frc.utils.alerts.PeriodicAlert;
import org.littletonrobotics.junction.Logger;

import java.util.LinkedList;

public abstract class Phoenix6Device implements IDevice {

	private final ConnectedInputAutoLogged connectedInput;
	private final String logPath;

	public Phoenix6Device(String logPath) {
		this.logPath = logPath;
		this.connectedInput = new ConnectedInputAutoLogged();
		connectedInput.connected = true;
		AlertManager.addAlert(new PeriodicAlert(Alert.AlertType.ERROR, logPath + "disconnectedAt", () -> !isConnected()));
	}

	public String getLogPath() {
		return logPath;
	}

	public boolean isConnected() {
		return connectedInput.connected;
	}


	private boolean isValid(InputSignal<?> signal) {
		return signal instanceof SignalGetter;
	}

	private void reportInvalidSignal(InputSignal<?> invalidSignal) {
		new Alert(
			Alert.AlertType.WARNING,
			logPath + "signal named " + invalidSignal.getName() + " has invalid type " + invalidSignal.getClass().getSimpleName()
		).report();
	}

	private InputSignal<?>[] getValidSignals(InputSignal<?>... signals) {
		LinkedList<InputSignal<?>> validSignals = new LinkedList<>();
		for (InputSignal<?> signal : signals) {
			if (isValid(signal)) {
				validSignals.add(signal);
			} else {
				reportInvalidSignal(signal);
			}
		}
		return validSignals.toArray(InputSignal<?>[]::new);
	}

	private void logSignals(InputSignal<?>... signals) {
		for (InputSignal<?> signal : signals) {
			Logger.processInputs(logPath, signal);
		}
	}

	@Override
	public void updateInputs(InputSignal<?>... inputSignals) {
		if (inputSignals.length == 0) {
			return;
		}
		InputSignal<?>[] validSignals = getValidSignals(inputSignals);
		connectedInput.connected = getDevice().isConnected();
		Logger.processInputs(logPath, connectedInput);
		logSignals(validSignals);
	}

	public abstract ParentDevice getDevice();

}
