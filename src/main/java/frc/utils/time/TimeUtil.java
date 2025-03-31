package frc.utils.time;

import edu.wpi.first.hal.HALUtil;
import frc.utils.Conversions;
import frc.utils.alerts.Alert;
import frc.utils.alerts.AlertManager;
import frc.utils.alerts.PeriodicAlert;
import org.littletonrobotics.junction.Logger;

public class TimeUtil {

	public static final double DEFAULT_CYCLE_TIME_SECONDS = 0.02;

	private static double lastCycleStartingTimeSeconds = 0;
	private static double currentCycleStartingTimeSeconds = 0;

	static {
		AlertManager.addAlert(
			new PeriodicAlert(
				Alert.AlertType.WARNING,
				TimeConstants.LOG_PATH + "/CycleOverrunAt",
				() -> getLatestCycleTimeSeconds() > DEFAULT_CYCLE_TIME_SECONDS + TimeConstants.TIME_STEP_TOLERANCE_SECONDS
			)
		);
	}

	public static void updateCycleTime(int roborioCycles) {
		lastCycleStartingTimeSeconds = currentCycleStartingTimeSeconds;
		currentCycleStartingTimeSeconds = getCurrentTimeSeconds();

		logStatus(roborioCycles);
	}

	private static void logStatus(int roborioCycles) {
		Logger.recordOutput(TimeConstants.LOG_PATH + "/CycleTimeSeconds", getLatestCycleTimeSeconds());
		Logger.recordOutput(TimeConstants.LOG_PATH + "/CurrentTimeSeconds", currentCycleStartingTimeSeconds);
		Logger.recordOutput(TimeConstants.LOG_PATH + "/AverageCycleTimeSeconds", currentCycleStartingTimeSeconds / roborioCycles);
	}

	public static double getCurrentTimeSeconds() {
		return Conversions.microSecondsToSeconds(HALUtil.getFPGATime());
	}

	public static double getLatestCycleTimeSeconds() {
		return currentCycleStartingTimeSeconds - lastCycleStartingTimeSeconds;
	}

}
