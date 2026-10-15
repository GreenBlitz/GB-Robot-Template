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
	private final String name;
	private final boolean isCritical;

	public Alert(AlertType type, String name, boolean isCritical) {
		this.logPath = ALERT_LOG_PATH + "/" + type.toString() + "/" + name;
		this.name = name;
		this.isCritical = isCritical;
	}

	public Alert(AlertType type, String name) {
		this.logPath = ALERT_LOG_PATH + "/" + type.toString() + "/" + name;
		this.name = name;
		this.isCritical = false;
	}

	public void report() {
		Logger.recordOutput(logPath, TimeUtil.getCurrentTimeSeconds());
		if (!AlertManager.getReportedAlerts().contains(this)) {
			AlertManager.getReportedAlerts().add(this);
		}
	}

	public String getName() {
		return name;
	}

	public boolean isCritical() {
		return isCritical;
	}

}
