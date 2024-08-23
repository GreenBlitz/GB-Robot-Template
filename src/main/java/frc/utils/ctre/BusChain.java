package frc.utils.ctre;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj.Timer;
import frc.utils.alerts.Alert;
import frc.utils.alerts.PeriodicAlert;
import org.littletonrobotics.junction.Logger;

public enum BusChain {

	ROBORIO("rio"),
	CANIVORE("CANivore");

	private final String chainName;
	private final String currentAlertLogPath;
	private CANBus.CANBusStatus busStatus;

	BusChain(String chainName) {
		this.chainName = chainName;
		this.busStatus = CANBus.getStatus(this.chainName);
		this.currentAlertLogPath = BusStatus.LOG_PATH + this.chainName;

		PeriodicAlert StatusError = new PeriodicAlert(
				Alert.AlertType.WARNING,
				currentAlertLogPath + "/StatusErrorAt",
				() -> !busStatus.Status.isOK()
		);
		PeriodicAlert Flooded = new PeriodicAlert(
				Alert.AlertType.WARNING,
				currentAlertLogPath + "/FloodedAt",
				() -> busStatus.BusUtilization > BusStatus.MAX_CAN_UTILIZATION_PERCENT
		);
		PeriodicAlert Disconnected = new PeriodicAlert(
				Alert.AlertType.ERROR,
				currentAlertLogPath + "/DisconnectedAt",
				() -> busStatus.BusOffCount > 0
		);
		PeriodicAlert Full = new PeriodicAlert(
				Alert.AlertType.ERROR,
				currentAlertLogPath + "/FullAt",
				() -> busStatus.TxFullCount > 0
		);
		PeriodicAlert ReceiveError = new PeriodicAlert(
				Alert.AlertType.WARNING,
				currentAlertLogPath + "/ReceiveErrorAt",
				() -> busStatus.REC > BusStatus.MAX_RECEIVE_ERRORS
		);
		PeriodicAlert TransmitErrors = new PeriodicAlert(
				Alert.AlertType.ERROR,
				currentAlertLogPath + "/TransmitErrorsAt",
				() -> busStatus.TEC > BusStatus.MAX_TRANSMIT_ERRORS
		);

		StatusError.addToAlertManager();
		Flooded.addToAlertManager();
		Disconnected.addToAlertManager();
		Full.addToAlertManager();
		ReceiveError.addToAlertManager();
		TransmitErrors.addToAlertManager();
	}

	public String getChainName() {
		return chainName;
	}

	void updateStatus() {
		this.busStatus = CANBus.getStatus(chainName);
		logStatus(busStatus);
	}

	private void logStatus(CANBus.CANBusStatus busStatus) {
		String currentLogPath = BusStatus.LOG_PATH + getChainName();
		Logger.recordOutput(currentLogPath + "/Status", busStatus.Status.getName());
		Logger.recordOutput(currentLogPath + "/Utilization", busStatus.BusUtilization);
		Logger.recordOutput(currentLogPath + "/TimesDisconnected", busStatus.BusOffCount);
		Logger.recordOutput(currentLogPath + "/Full", busStatus.TxFullCount);
		Logger.recordOutput(currentLogPath + "/ReceiveError", busStatus.REC);
		Logger.recordOutput(currentLogPath + "/TransmitError", busStatus.TEC);
	}

}
