package frc.utils;

import org.littletonrobotics.junction.Logger;

public class AlertLogger {

    public static void logAlert(String alertName, String alertDescription) {

        Logger.recordOutput("Alerts/"+alertName, alertDescription);
    }

    public static void logAlert(GBSubsystem subsystemName, String alertDescription) {

        Logger.recordOutput("Alerts/Subsystems/"+subsystemName.getName(), alertDescription);
    }

}
