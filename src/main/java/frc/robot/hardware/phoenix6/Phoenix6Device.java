package frc.robot.hardware.phoenix6;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import frc.robot.hardware.ConnectedInputAutoLogged;
import frc.robot.hardware.interfaces.IDevice;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6LatencyAndSlopeSignal;
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

	private StatusCode refreshSignals(InputSignal<?>... signals) {
		LinkedList<StatusSignal<?>> signalsSet = new LinkedList<>();
		for (InputSignal<?> signal : signals) {
			if (signal instanceof SignalGetter signalGetter) {
				signalsSet.add(signalGetter.getSignal());
				if (signal instanceof Phoenix6LatencyAndSlopeSignal bothLatencySignal) {
					signalsSet.add(bothLatencySignal.getSlopeSignal());
				}
			}
		}

		return BaseStatusSignal.refreshAll(signalsSet.toArray(StatusSignal[]::new));
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
//		refreshSignals(validSignals);
		logSignals(validSignals);
		connectedInput.connected = getDevice().isConnected();
		Logger.processInputs(logPath, connectedInput);
	}

	public abstract ParentDevice getDevice();

}
