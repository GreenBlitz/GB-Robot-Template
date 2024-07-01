package frc.utils.ctreutils;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.LogPathsConstants;
import org.littletonrobotics.junction.Logger;

public enum BusChain {

    CANBUS("rio"),
    CANIVORE("CANivore");

    private final String chainName;

    BusChain(String chainName){
        this.chainName = chainName;
    }

    public String getChainName() {
        return chainName;
    }

    void updateStatus() {
        CANBus.CANBusStatus busStatus = CANBus.getStatus(getChainName());
        logStatus(busStatus);
        reportAlerts(busStatus);
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

    private void reportAlerts(CANBus.CANBusStatus busStatus) {
        String currentAlertLogPath = LogPathsConstants.ALERT_LOG_PATH + BusStatus.LOG_PATH + getChainName();
        double currentTime = Timer.getFPGATimestamp();

        if (!busStatus.Status.isOK()) {
            Logger.recordOutput(currentAlertLogPath + "/StatusErrorAt", currentTime);
        }
        if (busStatus.BusUtilization > BusStatus.MAX_CAN_UTILIZATION_PERCENT) {
            Logger.recordOutput(currentAlertLogPath + "/FloodedAt", currentTime);
        }
        if (busStatus.BusOffCount > 0) {
            Logger.recordOutput(currentAlertLogPath + "/DisconnectedAt", currentTime);
        }
        if (busStatus.TxFullCount > 0) {
            Logger.recordOutput(currentAlertLogPath + "/FullAt", currentTime);
        }
        if (busStatus.REC > BusStatus.MAX_RECEIVE_ERRORS) {
            Logger.recordOutput(currentAlertLogPath + "/ReceiveErrorAt", currentTime);
        }
        if (busStatus.TEC > BusStatus.MAX_TRANSMIT_ERRORS) {
            Logger.recordOutput(currentAlertLogPath + "/TransmitErrorsAt", currentTime);
        }
    }

}
