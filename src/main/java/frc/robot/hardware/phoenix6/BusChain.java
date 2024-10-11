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

	private final String chainName;
	private final String logPath;
	private CANBusStatus busStatus;

	BusChain(String chainName) {
		this.chainName = chainName;
		this.logPath = LOG_PATH_PREFIX + this.chainName + "/";
		this.busStatus = CANBus.getStatus(this.chainName);

		createAlerts();
	}

	private void createAlerts() {
		//@formatter:off
		AlertManager.addAlert(
			new PeriodicAlert(
				Alert.AlertType.WARNING,
				logPath + "StatusErrorAt",
				() -> !busStatus.Status.isOK()
			)
		);
		AlertManager.addAlert(
			new PeriodicAlert(
				Alert.AlertType.WARNING,
				logPath + "ReceiveErrorAt",
				() -> busStatus.REC > PERMITTED_RECEIVE_ERRORS
			)
		);
		AlertManager.addAlert(
			new PeriodicAlert(
				Alert.AlertType.WARNING,
				logPath + "FloodedAt",
				() -> busStatus.BusUtilization > PERMITTED_CAN_UTILIZATION_DECIMAL_VALUE
			)
		);
		AlertManager.addAlert(
			new PeriodicAlert(
				Alert.AlertType.WARNING,
				logPath + "TransmitErrorsAt",
				() -> busStatus.TEC > PERMITTED_TRANSMIT_ERRORS
			)
		);
		AlertManager.addAlert(
			new PeriodicAlert(
				Alert.AlertType.ERROR,
				logPath + "DisconnectedAt",
				() -> busStatus.BusOffCount > PERMITTED_BUS_OFF_COUNT
			)
		);
		AlertManager.addAlert(
			new PeriodicAlert(
				Alert.AlertType.ERROR,
				logPath + "FullAt",
				() -> busStatus.TxFullCount > PERMITTED_TRANSMISSION_BUFFER_FULL_COUNT
			)
		);
		//@formatter:on
	}

	public String getChainName() {
		return chainName;
	}

	public void updateStatus() {
		busStatus = CANBus.getStatus(chainName);
		logStatus();
	}

	public void logStatus() {
		Logger.recordOutput(logPath + "Status", busStatus.Status.getName());
		Logger.recordOutput(logPath + "Utilization", busStatus.BusUtilization);
		Logger.recordOutput(logPath + "TimesDisconnected", busStatus.BusOffCount);
		Logger.recordOutput(logPath + "FullCount", busStatus.TxFullCount);
		Logger.recordOutput(logPath + "ReceiveError", busStatus.REC);
		Logger.recordOutput(logPath + "TransmitError", busStatus.TEC);
	}

	public static void logChainsStatuses() {
		for (BusChain chain : BusChain.values()) {
			chain.updateStatus();
		}
	}

}
