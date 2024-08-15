package frc.utils.alerts;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.LogPaths;
import org.littletonrobotics.junction.Logger;

public class Alert {

    public enum AlertType {

        ERROR,
        WARNING;

    }

    private final AlertType type;
    private final String logPath;
    private final String logPathAndTime;

    public Alert(AlertType type, String name) {
        this.type = type;
        this.logPath = LogPaths.ALERT_LOG_PATH + type.toString() + "/" + name;
        this.logPathAndTime = logPath + " at time: " + Timer.getFPGATimestamp();
    }

    public void logAlert() {
        Logger.recordOutput(logPath, Timer.getFPGATimestamp());
        if (type == AlertType.WARNING) {
            DriverStation.reportWarning(logPathAndTime, false);
        } else {
            DriverStation.reportError(logPathAndTime, false);
        }
    }

}
