package frc.utils.CTREUtils;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.LogPathsConstants;
import frc.robot.constants.Phoenix6Constants;
import org.littletonrobotics.junction.Logger;

public class CANStatus {

    private static final double MAX_CAN_UTILIZATION_PERCENT = 0.6;
    private static final double MAX_RECEIVE_ERRORS = 1;
    private static final double MAX_TRANSMIT_ERRORS = 1;
    private static final String LOG_PATH = "Bus/";

    public static void logAllBusStatuses() {
        updateBusStatus(Phoenix6Constants.CANBUS_NAME);
        updateBusStatus(Phoenix6Constants.CANIVORE_NAME);
    }

    private static void updateBusStatus(String name) {
        CANBus.CANBusStatus busStatus = CANBus.getStatus(name);
        logBusStatus(busStatus, name);
        reportBusAlerts(busStatus, name);
    }

    private static void logBusStatus(CANBus.CANBusStatus busStatus, String name) {
        Logger.recordOutput(LOG_PATH + name + "/Status", busStatus.Status.getName());
        Logger.recordOutput(LOG_PATH + name + "/Utilization", busStatus.BusUtilization);
        Logger.recordOutput(LOG_PATH + name + "/TimesDisconnected", busStatus.BusOffCount);
        Logger.recordOutput(LOG_PATH + name + "/Full", busStatus.TxFullCount);
        Logger.recordOutput(LOG_PATH + name + "/ReceiveError", busStatus.REC);
        Logger.recordOutput(LOG_PATH + name + "/TransmitError", busStatus.TEC);
    }

    private static void reportBusAlerts(CANBus.CANBusStatus busStatus, String name) {
        String currentAlertLogPath = LogPathsConstants.ALERT_LOG_PATH + LOG_PATH + name;
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
