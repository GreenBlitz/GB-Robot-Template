package frc.robot.hardware;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import frc.robot.hardware.signal.InputSignal;
import frc.robot.hardware.signal.phoenix.Phoenix6BothLatencySignal;
import frc.robot.hardware.signal.phoenix.Phoenix6SignalBuilder;
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
		AlertManager.addAlert(new PeriodicAlert(Alert.AlertType.WARNING, logPath, () -> !isConnected()));
	}

	public boolean isConnected() {
		return connectedInput.connected;
	}

	public void fetchSignals(InputSignal... signals) {
		LinkedList<StatusSignal<Double>> signalsSet = new LinkedList<>();
		for (InputSignal signal : signals) {
			if (signal instanceof Phoenix6SignalBuilder.SignalGetter) {
				signalsSet.add(((Phoenix6SignalBuilder.SignalGetter) signal).getSignal());
				if (signal instanceof Phoenix6BothLatencySignal) {
					signalsSet.add(((Phoenix6BothLatencySignal) signal).getSignalSlope());
				}
			}
		}

		connectedInput.connected = BaseStatusSignal.refreshAll(signalsSet.toArray(StatusSignal[]::new)).isOK();
		Logger.processInputs(logPath, connectedInput);

		for (InputSignal signal : signals) {
			Logger.processInputs(logPath, signal);
		}
	}

}
