package frc.utils;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.LogPaths;
import org.littletonrobotics.junction.Logger;

public class Alert {

    public enum AlertType {
        ERROR,
        WARNING;
    }

    private final AlertType type;
    private final String name;

    public Alert(AlertType alertType, String alertName){
        this.type = alertType;
        this.name = alertName;
    }

    public void logAlert(){
        Logger.recordOutput(LogPaths.ALERT_LOG_PATH +type.toString()+"/"+name, Timer.getFPGATimestamp());
    }

    public void logAlert(GBSubsystem subsystem){
        Logger.recordOutput(LogPaths.ALERT_LOG_PATH+subsystem.toString()+"/"+type.toString()+"/"+name, Timer.getFPGATimestamp());
    }

}