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
	private double lastReportedTime;
	private int timesHappenedBetweenReports;

	public Alert(AlertType type, String name) {
		this.type = type;
		this.logPath = ALERT_LOG_PATH + type.toString() + "/" + name;
		this.lastReportedTime = -SECONDS_BETWEEN_REPORTS;
		this.timesHappenedBetweenReports = 0;
	}

	public void report() {
		if (!DriverStationUtils.isMatch()) {
			switch (type) {
				case ERROR:
					if (lastReportedTime <= TimeUtils.getCurrentTimeSeconds() - SECONDS_BETWEEN_REPORTS) {
						DriverStation.reportError(
							logPath + " happened " + timesHappenedBetweenReports + 1 + " in the last " + SECONDS_BETWEEN_REPORTS + " seconds.",
							LOG_TRACE
						);
						lastReportedTime = TimeUtils.getCurrentTimeSeconds();
						timesHappenedBetweenReports = 0;
					} else {
						timesHappenedBetweenReports++;
					}
			}
		}
		Logger.recordOutput(logPath, TimeUtils.getCurrentTimeSeconds());
	}

}
