package frc.utils;

import org.littletonrobotics.junction.Logger;

public class Alert {

    private AlertType type;
    private String name;
    private String description;
    private static final String directory = "Alerts/";

    public enum AlertType {
        ERROR,
        WARNING,
        LOG;
    }

    public Alert(AlertType alertType, String alertName, String alertDescription){

        this.type = alertType;
        this.name = alertName;
        this.description = alertDescription;
    }

    public void logAlert(){
        Logger.recordOutput(directory+type.toString()+"/"+name, description);
    }

    public void logAlert(GBSubsystem subsystem){
        Logger.recordOutput(directory+subsystem.toString()+"/"+type.toString()+"/"+name, description);
    }


}