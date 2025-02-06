package frc.robot.hardware.phoenix6;

import com.ctre.phoenix6.CANBus;
import frc.utils.alerts.Alert;
import frc.utils.alerts.AlertManager;
import frc.utils.alerts.PeriodicAlert;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.CANBus.CANBusStatus;

public enum BusChain {

	ROBORIO(""),
	CANIVORE("*");

	private static final double PERMITTED_CAN_UTILIZATION_DECIMAL_VALUE = 0.6;
	private static final int PERMITTED_RECEIVE_ERRORS = 0;
	private static final int PERMITTED_TRANSMIT_ERRORS = 0;
	private static final String LOG_PATH_PREFIX = "Bus";

	private final CANBus canBus;
	private final String logPath;
	private CANBusStatus currentBusStatus;
	private CANBusStatus lastBusStatus;

	BusChain(String chainName) {
		this.canBus = new CANBus(chainName);
		this.logPath = LOG_PATH_PREFIX + "/" + getChainName();
		this.currentBusStatus = canBus.getStatus();
		this.lastBusStatus = new CANBusStatus();

		createAlerts();
	}

	private void createAlerts() {
		//@formatter:off
		AlertManager.addAlert(
			new PeriodicAlert(
				Alert.AlertType.WARNING,
				logPath + "/StatusErrorAt",
				() -> !currentBusStatus.Status.isOK()
			)
		);
		AlertManager.addAlert(
			new PeriodicAlert(
				Alert.AlertType.WARNING,
				logPath + "/ReceiveErrorAt",
				() -> currentBusStatus.REC > PERMITTED_RECEIVE_ERRORS
			)
		);
		AlertManager.addAlert(
			new PeriodicAlert(
				Alert.AlertType.WARNING,
				logPath + "/FloodedAt",
				() -> currentBusStatus.BusUtilization > PERMITTED_CAN_UTILIZATION_DECIMAL_VALUE
			)
		);
		AlertManager.addAlert(
			new PeriodicAlert(
				Alert.AlertType.WARNING,
				logPath + "/TransmitErrorsAt",
				() -> currentBusStatus.TEC > PERMITTED_TRANSMIT_ERRORS
			)
		);

		PeriodicAlert busOffAlert = new PeriodicAlert(
			Alert.AlertType.ERROR,
			logPath + "/BusOffAt",
			() -> currentBusStatus.BusOffCount > lastBusStatus.BusOffCount
		);
		busOffAlert.reportByCondition();
		AlertManager.addAlert(busOffAlert);

		PeriodicAlert busFullAlert = new PeriodicAlert(
			Alert.AlertType.ERROR,
			logPath + "/FullAt",
			() -> currentBusStatus.TxFullCount > lastBusStatus.TxFullCount
		);
		busFullAlert.reportByCondition();
		AlertManager.addAlert(busFullAlert);
		//@formatter:on
	}

	public String getChainName() {
		return canBus.getName();
	}

	public void updateStatus() {
		lastBusStatus = copyStatus(currentBusStatus);
		currentBusStatus = canBus.getStatus();
		logStatus();
	}

	public void logStatus() {
		Logger.recordOutput(logPath + "/Status", currentBusStatus.Status.getName());
		Logger.recordOutput(logPath + "/Utilization", currentBusStatus.BusUtilization);
		Logger.recordOutput(logPath + "/TimesDisconnected", currentBusStatus.BusOffCount);
		Logger.recordOutput(logPath + "/FullCount", currentBusStatus.TxFullCount);
		Logger.recordOutput(logPath + "/ReceiveError", currentBusStatus.REC);
		Logger.recordOutput(logPath + "/TransmitError", currentBusStatus.TEC);
	}

	public static void logChainsStatuses() {
		for (BusChain chain : BusChain.values()) {
			chain.updateStatus();
		}
	}

	public static CANBusStatus copyStatus(CANBusStatus toCopy) {
		CANBusStatus copiedBusStatus = new CANBusStatus();
		copiedBusStatus.Status = toCopy.Status;
		copiedBusStatus.BusUtilization = toCopy.BusUtilization;
		copiedBusStatus.BusOffCount = toCopy.BusOffCount;
		copiedBusStatus.TxFullCount = toCopy.TxFullCount;
		copiedBusStatus.REC = toCopy.REC;
		copiedBusStatus.TEC = toCopy.TEC;
		return copiedBusStatus;
	}

}
