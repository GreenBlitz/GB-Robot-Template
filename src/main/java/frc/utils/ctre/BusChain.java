package frc.utils.ctre;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.LogPaths;
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
	}

	public String getChainName() {
		return chainName;
	}

	public void updateStatus() {
		busStatus = CANBus.getStatus(chainName);
		logStatus();
		reportAlerts();
	}

	public void logStatus() {
		Logger.recordOutput(logPath + "Status", busStatus.Status.getName());
		Logger.recordOutput(logPath + "Utilization", busStatus.BusUtilization);
		Logger.recordOutput(logPath + "TimesDisconnected", busStatus.BusOffCount);
		Logger.recordOutput(logPath + "FullCount", busStatus.TxFullCount);
		Logger.recordOutput(logPath + "ReceiveError", busStatus.REC);
		Logger.recordOutput(logPath + "TransmitError", busStatus.TEC);
	}

	public void reportAlerts() {
		String alertLogPath = LogPaths.ALERT_LOG_PATH + logPath;
		double currentTime = Timer.getFPGATimestamp();

		if (!busStatus.Status.isOK()) {
			Logger.recordOutput(alertLogPath + "StatusErrorAt", currentTime);
		}
		if (busStatus.BusUtilization > PERMITTED_CAN_UTILIZATION_DECIMAL_VALUE) {
			Logger.recordOutput(alertLogPath + "FloodedAt", currentTime);
		}
		if (busStatus.BusOffCount > PERMITTED_BUS_OFF_COUNT) {
			Logger.recordOutput(alertLogPath + "DisconnectedAt", currentTime);
		}
		if (busStatus.TxFullCount > PERMITTED_TRANSMISSION_BUFFER_FULL_COUNT) {
			Logger.recordOutput(alertLogPath + "FullAt", currentTime);
		}
		if (busStatus.REC > PERMITTED_RECEIVE_ERRORS) {
			Logger.recordOutput(alertLogPath + "ReceiveErrorAt", currentTime);
		}
		if (busStatus.TEC > PERMITTED_TRANSMIT_ERRORS) {
			Logger.recordOutput(alertLogPath + "TransmitErrorsAt", currentTime);
		}
	}

	public static void logChainsStatuses() {
		for (BusChain chain : BusChain.values()) {
			chain.updateStatus();
		}
	}

}
