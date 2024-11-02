package frc.robot.hardware.phoenix6;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import frc.robot.hardware.ConnectedInputAutoLogged;
import frc.robot.hardware.IDevice;
import frc.robot.hardware.signal.InputSignal;
import frc.robot.hardware.signal.phoenix6.Phoenix6BothLatencySignal;
import frc.robot.hardware.signal.phoenix6.Phoenix6SignalBuilder;
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

	public boolean isConnected() {
		return connectedInput.connected;
	}


	private boolean isValid(InputSignal<?> signal) {
		return signal instanceof Phoenix6SignalBuilder.SignalGetter;
	}

	private void reportInvalidSignal(InputSignal<?> invalidSignal) {
		new Alert(
			Alert.AlertType.WARNING,
			logPath + "signal named " + invalidSignal.getName() + " got invalid type " + invalidSignal.getClass().getSimpleName()
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
		LinkedList<StatusSignal<Double>> signalsSet = new LinkedList<>();
		for (InputSignal<?> signal : signals) {
			if (signal instanceof Phoenix6SignalBuilder.SignalGetter signalGetter) {
				signalsSet.add(signalGetter.getSignal());
				if (signal instanceof Phoenix6BothLatencySignal bothLatencySignal) {
					signalsSet.add(bothLatencySignal.getSignalSlope());
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
	public void updateSignals(InputSignal<?>... signals) {
		InputSignal<?>[] validSignals = getValidSignals(signals);
		connectedInput.connected = refreshSignals(validSignals).isOK();
		Logger.processInputs(logPath, connectedInput);
		logSignals(validSignals);
	}

}
