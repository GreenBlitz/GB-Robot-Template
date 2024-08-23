package frc.utils.alerts;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

public class Alert {

	public enum AlertType {

		ERROR,
		PLACEHOLDER,
		WARNING;

	}

	private static final boolean LOG_TRACE = false;
	public static final String ALERT_LOG_PATH = "Alerts/";
	private final AlertType type;
	private final String logPath;

	public Alert(AlertType type, String name) {
		this.type = type;
		this.logPath = ALERT_LOG_PATH + type.toString() + "/" + name;
	}

	public void reportAlert() {
		switch (type) {
			case ERROR -> DriverStation.reportError(logPath, LOG_TRACE);
		}
		Logger.recordOutput(logPath, Timer.getFPGATimestamp());
	}

}
