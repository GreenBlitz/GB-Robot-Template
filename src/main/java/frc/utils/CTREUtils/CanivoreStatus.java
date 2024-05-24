package frc.utils.CTREUtils;

import com.ctre.phoenix6.CANBus;
import frc.robot.constants.LogPathsConstants;
import frc.robot.constants.Phoenix6Constants;
import org.littletonrobotics.junction.Logger;

public class CanivoreStatus {

    private static final double MAX_CAN_UTILIZATION_PERCENT = 90;
    private static final double MAX_RECEIVE_ERRORS = 3;
    private static final double MAX_TRANSMIT_ERRORS = 3;
    private static final String LOG_PATH = "Bus/";

    public static void logAllBusStatuses() {
        updateBusStatus(Phoenix6Constants.CANBUS_NAME);
        updateBusStatus(Phoenix6Constants.CANIVORE_NAME);
    }

    private static void updateBusStatus(String name){
        var busStatus = CANBus.getStatus(name);
        logBusStatus(busStatus, name);
        reportBusAlerts(busStatus, name);
    }

    private static void logBusStatus(CANBus.CANBusStatus busStatus, String name) {
        Logger.recordOutput(LOG_PATH + name + "/Status", busStatus.Status.getName());
        Logger.recordOutput(LOG_PATH + name + "/Utilization", busStatus.BusUtilization);
        Logger.recordOutput(LOG_PATH + name + "/TimesDisconnected", busStatus.BusOffCount);
        Logger.recordOutput(LOG_PATH + name + "/Full", busStatus.TxFullCount);
        Logger.recordOutput(LOG_PATH + name + "/ReceiveErrorCount", busStatus.REC);
        Logger.recordOutput(LOG_PATH + name + "/TransmitErrorCount", busStatus.TEC);
    }

    private static void reportBusAlerts(CANBus.CANBusStatus busStatus, String name) {
        if (!busStatus.Status.isOK()) {
            Logger.recordOutput(LogPathsConstants.ALERT_LOG_PATH + LOG_PATH + name + "/Status", busStatus.Status.getName());
        }
        if (busStatus.BusUtilization > MAX_CAN_UTILIZATION_PERCENT) {
            Logger.recordOutput(LogPathsConstants.ALERT_LOG_PATH + LOG_PATH + name + "/FloodedAt", busStatus.BusUtilization);
        }
        if (busStatus.BusOffCount > 0) {
            Logger.recordOutput(LogPathsConstants.ALERT_LOG_PATH + LOG_PATH + name + "/DisconnectedAt", busStatus.BusOffCount);
        }
        if (busStatus.TxFullCount > 0) {
            Logger.recordOutput(LogPathsConstants.ALERT_LOG_PATH + LOG_PATH + name + "/FullAt", busStatus.TxFullCount);
        }
        if (busStatus.REC > MAX_RECEIVE_ERRORS) {
            Logger.recordOutput(LogPathsConstants.ALERT_LOG_PATH + LOG_PATH + name + "/MaxReceiveErrors", busStatus.REC);
        }
        if (busStatus.REC > MAX_TRANSMIT_ERRORS) {
            Logger.recordOutput(LogPathsConstants.ALERT_LOG_PATH + LOG_PATH + name + "/MaxTransmitErrors", busStatus.TEC);
        }
    }

}
