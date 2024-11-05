package frc.utils.time;

import edu.wpi.first.hal.HALUtil;
import frc.robot.RobotManager;
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

	public static void updateCycleTime(double roborioCycles) {
		lastCycleTimeSeconds = newCycleTimeSeconds;
		newCycleTimeSeconds = getCurrentTimeSeconds();

		logStatus(roborioCycles);
	}

	private static void logStatus(double roborioCycles) {
		double currentTimeSeconds = getCurrentTimeSeconds();

		Logger.recordOutput(TimeConstants.LOG_PATH + "CycleTimeSeconds", getCurrentCycleTimeSeconds());
		Logger.recordOutput(TimeConstants.LOG_PATH + "CurrentTimeSeconds", currentTimeSeconds);
		Logger.recordOutput(TimeConstants.LOG_PATH + "AverageCycleTimeSeconds", currentTimeSeconds / roborioCycles);
	}

	public static double getCurrentTimeSeconds() {
		return Conversions.microSecondsToSeconds(HALUtil.getFPGATime());
	}

	public static double getCurrentCycleTimeSeconds() {
		return newCycleTimeSeconds - lastCycleTimeSeconds;
	}

}
