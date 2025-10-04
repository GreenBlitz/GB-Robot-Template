package frc.utils.alerts;

import frc.utils.time.TimeUtil;
import org.littletonrobotics.junction.Logger;

public class Alert {

	public enum AlertType {

		ERROR,
		WARNING;

	}

	private static final String ALERT_LOG_PATH = "Alerts";

	private final String logPath;

	public Alert(AlertType type, String name) {
		this.logPath = ALERT_LOG_PATH + "/" + type.toString() + "/" + name;
	}

	public void report() {
		Logger.recordOutput(logPath, TimeUtil.getCurrentTimeSeconds());
	}

}
