package frc.utils.ctreutils;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.LogPathsConstants;
import frc.robot.constants.Phoenix6Constants;
import org.littletonrobotics.junction.Logger;


public class BusStatus {

    private static final double MAX_CAN_UTILIZATION_PERCENT = 0.6;
    private static final double MAX_RECEIVE_ERRORS = 0;
    private static final double MAX_TRANSMIT_ERRORS = 0;
    private static final String LOG_PATH = "Bus/";

    public static void logChainsStatuses() {
        updateChainStatus(Phoenix6Constants.CANBUS_NAME);
        updateChainStatus(Phoenix6Constants.CANIVORE_NAME);
    }

    private static void updateChainStatus(String chainName) {
        CANBus.CANBusStatus busStatus = CANBus.getStatus(chainName);
        logStatus(busStatus, chainName);
        reportAlerts(busStatus, chainName);
    }

    private static void logStatus(CANBus.CANBusStatus busStatus, String chainName) {
        String currentLogPath = LOG_PATH + chainName;
        Logger.recordOutput(currentLogPath + "/Status", busStatus.Status.getName());
        Logger.recordOutput(currentLogPath + "/Utilization", busStatus.BusUtilization);
        Logger.recordOutput(currentLogPath + "/TimesDisconnected", busStatus.BusOffCount);
        Logger.recordOutput(currentLogPath + "/Full", busStatus.TxFullCount);
        Logger.recordOutput(currentLogPath + "/ReceiveError", busStatus.REC);
        Logger.recordOutput(currentLogPath + "/TransmitError", busStatus.TEC);
    }

    private static void reportAlerts(CANBus.CANBusStatus busStatus, String chainName) {
        String currentAlertLogPath = LogPathsConstants.ALERT_LOG_PATH + LOG_PATH + chainName;
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
