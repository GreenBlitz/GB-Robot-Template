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

    private static final boolean LOG_TRACE = false;
    private final AlertType type;
    private final String logPath;

    public Alert(AlertType type, String name) {
        this.type = type;
        this.logPath = LogPaths.ALERT_LOG_PATH + type.toString() + "/" + name;
    }

    public void logAlert() {
        Logger.recordOutput(logPath, Timer.getFPGATimestamp());
        switch (type) {
            case WARNING -> DriverStation.reportWarning(logPath, LOG_TRACE);
            case ERROR -> DriverStation.reportError(logPath, LOG_TRACE);
        }
    }

}
