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
	private static final double SECONDS_BETWEEN_REPORTS = 3;
	private final AlertType type;
	private final String logPath;
	private double lastReportedTimeSeconds;
	private int timesOccurredSinceLastReport;

	public Alert(AlertType type, String name) {
		this.type = type;
		this.logPath = ALERT_LOG_PATH + type.toString() + "/" + name;
		this.lastReportedTimeSeconds = 0;
		this.timesOccurredSinceLastReport = 0;
	}
	
	private boolean shouldReportToDriverStation() {
		return lastReportedTimeSeconds <= TimeUtils.getCurrentTimeSeconds() - SECONDS_BETWEEN_REPORTS;
	}
	
	private void reportToDriverStation() {
		DriverStation.reportError(
				logPath + " happened " + timesOccurredSinceLastReport + " in the last " + SECONDS_BETWEEN_REPORTS + " seconds.",
				LOG_TRACE
		);
		lastReportedTimeSeconds = TimeUtils.getCurrentTimeSeconds();
		timesOccurredSinceLastReport = 0;
	}

	public void report() {
		timesOccurredSinceLastReport++;
		if (!DriverStationUtils.isMatch()) {
			switch (type) {
				case ERROR:
					if (shouldReportToDriverStation()) {
						reportToDriverStation();
					}
			}
		}
		Logger.recordOutput(logPath, TimeUtils.getCurrentTimeSeconds());
	}

}
