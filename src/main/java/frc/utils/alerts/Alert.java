package frc.utils.alerts;

import edu.wpi.first.wpilibj.DriverStation;
import frc.utils.DriverStationUtils;
import frc.utils.time.TimeUtils;
import org.littletonrobotics.junction.Logger;

public class Alert {

	public enum AlertType {

		ERROR,
		WARNING;

	}

	private static final boolean LOG_TRACE = false;
	private static final String ALERT_LOG_PATH = "Alerts/";
	private final AlertType type;
	private final String logPath;

	public Alert(AlertType type, String name) {
		this.type = type;
		this.logPath = ALERT_LOG_PATH + type.toString() + "/" + name;
	}

	public void report() {
		if (!DriverStationUtils.isMatch() && false) {
			switch (type) {
				case ERROR -> DriverStation.reportError(logPath, LOG_TRACE);
			}
		}
		Logger.recordOutput(logPath, TimeUtils.getCurrentTimeSeconds());
	}

}
