package frc.utils;

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
    private final String name;
	private final String logKey;
	private final String logKeyAndTime;

    public Alert(AlertType type, String name) {
        this.type = type;
        this.name = name;
		logKey = LogPaths.ALERT_LOG_PATH + type.toString() + "/" + name;
		logKeyAndTime = logKey + " at time: " + Timer.getFPGATimestamp();
    }

    public void logAlert() {
        Logger.recordOutput(logKey, Timer.getFPGATimestamp());
		DriverStation.reportWarning(logKeyAndTime, false);
    }

}
