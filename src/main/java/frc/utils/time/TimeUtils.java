package frc.utils.time;

import edu.wpi.first.hal.HALUtil;
import frc.utils.Conversions;
import frc.utils.alerts.Alert;
import frc.utils.alerts.AlertManager;
import frc.utils.alerts.PeriodicAlert;
import org.littletonrobotics.junction.Logger;

public class TimeUtils {

	public static final double DEFAULT_CYCLE_TIME_SECONDS = 0.02;

	private static double lastCycleTimeSeconds = 0;
	private static double newCycleTimeSeconds = 0;

	static {
		AlertManager.addAlert(
			new PeriodicAlert(
				Alert.AlertType.WARNING,
				TimeConstants.LOG_PATH + "CycleOverrunAt",
				() -> getCurrentCycleTimeSeconds() > DEFAULT_CYCLE_TIME_SECONDS + TimeConstants.TIME_STEP_TOLERANCE_SECONDS
			)
		);
	}

	public static void updateCycleTime() {
		lastCycleTimeSeconds = newCycleTimeSeconds;
		newCycleTimeSeconds = getCurrentTimeSeconds();

		logStatus();
	}



	private static void logStatus() {
		Logger.recordOutput(TimeConstants.LOG_PATH + "CycleTime", getCurrentCycleTimeSeconds());
	}

	public static double getCurrentTimeSeconds() {
		return Conversions.microSecondsToSeconds(HALUtil.getFPGATime());
	}

	public static double getCurrentCycleTimeSeconds() {
		return newCycleTimeSeconds - lastCycleTimeSeconds;
	}

}
