package frc.utils.ctre;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.LogPaths;
import org.littletonrobotics.junction.Logger;

public enum BusChain {

	ROBORIO("rio"),
	CANIVORE("CANivore");

	private static final double MAX_CAN_UTILIZATION_PERCENT = 0.6;
	private static final double MAX_RECEIVE_ERRORS = 0;
	private static final double MAX_TRANSMIT_ERRORS = 0;
	private static final String LOG_PATH = "Bus/";

	private final String chainName;
	private final String logPath;
	private CANBus.CANBusStatus busStatus;

	BusChain(String chainName) {
		this.chainName = chainName;
		this.logPath = LOG_PATH + getChainName();
		this.busStatus = CANBus.getStatus(getChainName());
	}

	public String getChainName() {
		return chainName;
	}

	public void updateStatus() {
		busStatus = CANBus.getStatus(getChainName());
		logStatus();
		reportAlerts();
	}

	public void logStatus() {
		String currentLogPath = logPath;
		Logger.recordOutput(currentLogPath + "/Status", busStatus.Status.getName());
		Logger.recordOutput(currentLogPath + "/Utilization", busStatus.BusUtilization);
		Logger.recordOutput(currentLogPath + "/TimesDisconnected", busStatus.BusOffCount);
		Logger.recordOutput(currentLogPath + "/Full", busStatus.TxFullCount);
		Logger.recordOutput(currentLogPath + "/ReceiveError", busStatus.REC);
		Logger.recordOutput(currentLogPath + "/TransmitError", busStatus.TEC);
	}

	public void reportAlerts() {
		String currentAlertLogPath = LogPaths.ALERT_LOG_PATH + LOG_PATH + getChainName();
		double currentTime = Timer.getFPGATimestamp();

		if (!busStatus.Status.isOK()) {
			Logger.recordOutput(currentAlertLogPath + "/StatusErrorAt", currentTime);
		}
		if (busStatus.BusUtilization > MAX_CAN_UTILIZATION_PERCENT) {
			Logger.recordOutput(currentAlertLogPath + "/FloodedAt", currentTime);
		}
		if (busStatus.BusOffCount > 0) {
			Logger.recordOutput(currentAlertLogPath + "/DisconnectedAt", currentTime);
		}
		if (busStatus.TxFullCount > 0) {
			Logger.recordOutput(currentAlertLogPath + "/FullAt", currentTime);
		}
		if (busStatus.REC > MAX_RECEIVE_ERRORS) {
			Logger.recordOutput(currentAlertLogPath + "/ReceiveErrorAt", currentTime);
		}
		if (busStatus.TEC > MAX_TRANSMIT_ERRORS) {
			Logger.recordOutput(currentAlertLogPath + "/TransmitErrorsAt", currentTime);
		}
	}

}
