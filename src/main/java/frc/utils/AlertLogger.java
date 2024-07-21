package frc.utils;

import org.littletonrobotics.junction.Logger;

public class AlertLogger {

    public enum AlertType {
        ERROR,
        INFO;
    }

    public static void logAlert(AlertType alertType, String alertName, String alertDescription) {

        Logger.recordOutput("Alerts/"+alertType.toString()+"/"+alertName, alertDescription);
    }

    public static void logAlert(AlertType alertType, GBSubsystem subsystemName, String alertDescription) {

        Logger.recordOutput("Alerts/"+alertType.toString()+"/Subsystems/"+subsystemName.getName(), alertDescription);
    }

}
