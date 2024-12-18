package frc.robot.hardware.phoenix6;

import com.ctre.phoenix6.CANBus;
import frc.utils.alerts.Alert;
import frc.utils.alerts.AlertManager;
import frc.utils.alerts.PeriodicAlert;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.CANBus.CANBusStatus;

public enum BusChain {

	ROBORIO("rio"),
	CANIVORE("CANivore");

	private static final double PERMITTED_CAN_UTILIZATION_DECIMAL_VALUE = 0.6;
	private static final int PERMITTED_RECEIVE_ERRORS = 0;
	private static final int PERMITTED_TRANSMIT_ERRORS = 0;
	private static final int PERMITTED_TRANSMISSION_BUFFER_FULL_COUNT = 0;
	private static final int PERMITTED_BUS_OFF_COUNT = 0;
	private static final String LOG_PATH_PREFIX = "Bus/";

	private final CANBus canBus;
	private final String logPath;
	private CANBusStatus currentBusStatus;
	private CANBusStatus lastBusStatus;

	BusChain(String chainName) {
		this.canBus = new CANBus(chainName);
		this.logPath = LOG_PATH_PREFIX + getChainName() + "/";
		this.currentBusStatus = canBus.getStatus();
		this.lastBusStatus = currentBusStatus;

		createAlerts();
	}

	private void createAlerts() {
		//@formatter:off
		AlertManager.addAlert(
			new PeriodicAlert(
				Alert.AlertType.WARNING,
				logPath + "StatusErrorAt",
				() -> !currentBusStatus.Status.isOK()
			)
		);
		AlertManager.addAlert(
			new PeriodicAlert(
				Alert.AlertType.WARNING,
				logPath + "ReceiveErrorAt",
				() -> currentBusStatus.REC > PERMITTED_RECEIVE_ERRORS
			)
		);
		AlertManager.addAlert(
			new PeriodicAlert(
				Alert.AlertType.WARNING,
				logPath + "FloodedAt",
				() -> currentBusStatus.BusUtilization > PERMITTED_CAN_UTILIZATION_DECIMAL_VALUE
			)
		);
		AlertManager.addAlert(
			new PeriodicAlert(
				Alert.AlertType.WARNING,
				logPath + "TransmitErrorsAt",
				() -> currentBusStatus.TEC > PERMITTED_TRANSMIT_ERRORS
			)
		);
		AlertManager.addAlert(
			new PeriodicAlert(
				Alert.AlertType.ERROR,
				logPath + "DisconnectedAt",
				() -> currentBusStatus.BusOffCount > lastBusStatus.BusOffCount
			)
		);
		AlertManager.addAlert(
			new PeriodicAlert(
				Alert.AlertType.ERROR,
				logPath + "FullAt",
				() -> currentBusStatus.TxFullCount > lastBusStatus.TxFullCount
			)
		);
		//@formatter:on
	}

	public String getChainName() {
		return canBus.getName();
	}

	public void updateStatus() {
		lastBusStatus = currentBusStatus;
		currentBusStatus = canBus.getStatus();
		logStatus();
	}

	public void logStatus() {
		Logger.recordOutput(logPath + "Status", currentBusStatus.Status.getName());
		Logger.recordOutput(logPath + "Utilization", currentBusStatus.BusUtilization);
		Logger.recordOutput(logPath + "TimesDisconnected", currentBusStatus.BusOffCount);
		Logger.recordOutput(logPath + "FullCount", currentBusStatus.TxFullCount);
		Logger.recordOutput(logPath + "ReceiveError", currentBusStatus.REC);
		Logger.recordOutput(logPath + "TransmitError", currentBusStatus.TEC);
	}

	public static void logChainsStatuses() {
		for (BusChain chain : BusChain.values()) {
			chain.updateStatus();
		}
	}

}
