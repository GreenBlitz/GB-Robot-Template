package frc.utils.alerts;

import edu.wpi.first.wpilibj.DriverStation;
import frc.utils.DriverStationUtil;
import frc.utils.time.TimeUtil;
import org.littletonrobotics.junction.Logger;

public class Alert {

	public enum AlertType {

		ERROR,
		WARNING;

	}

	private static final boolean LOG_TRACE = false;
	private static final String ALERT_LOG_PATH = "Alerts";
	private static final double SECONDS_BETWEEN_REPORTS = 3;

	private final AlertType type;
	private final String logPath;
	private double lastTimeReportedToDriveStationSeconds;
	private int timesOccurredSinceLastReportToDriverStation;

	public Alert(AlertType type, String name) {
		this.type = type;
		this.logPath = ALERT_LOG_PATH + "/" + type.toString() + "/" + name;
		this.lastTimeReportedToDriveStationSeconds = 0;
		this.timesOccurredSinceLastReportToDriverStation = 0;
	}

	private boolean shouldReportToDriverStation() {
		return lastTimeReportedToDriveStationSeconds + SECONDS_BETWEEN_REPORTS <= TimeUtil.getCurrentTimeSeconds();
	}

	private void reportToDriverStation() {
		DriverStation.reportError(
			logPath + " happened " + timesOccurredSinceLastReportToDriverStation + " in the last " + SECONDS_BETWEEN_REPORTS + " seconds.",
			LOG_TRACE
		);
		lastTimeReportedToDriveStationSeconds = TimeUtil.getCurrentTimeSeconds();
		timesOccurredSinceLastReportToDriverStation = 0;
	}

	public void report() {
		timesOccurredSinceLastReportToDriverStation++;
		if (!DriverStationUtil.isMatch()) {
			switch (type) {
				case ERROR:
					if (shouldReportToDriverStation()) {
						reportToDriverStation();
					}
			}
		}
		Logger.recordOutput(logPath, TimeUtil.getCurrentTimeSeconds());
	}

}
