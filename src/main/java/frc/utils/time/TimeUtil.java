package frc.utils.time;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.RobotManager;
import frc.robot.Robot;
import frc.utils.GamePeriodUtils;
import frc.utils.alerts.Alert;
import frc.utils.alerts.AlertManager;
import frc.utils.alerts.PeriodicAlert;
import org.littletonrobotics.junction.Logger;

public class TimeUtil {

	public static final double DEFAULT_CYCLE_TIME_SECONDS = 0.02;

	private static double lastCycleStartingTimeSeconds = 0;
	private static double currentCycleStartingTimeSeconds = 0;
	private static double autonomousStartTimeSeconds = 0;

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
		return Timer.getTimestamp();
	}

	public static double getLatestCycleTimeSeconds() {
		return currentCycleStartingTimeSeconds - lastCycleStartingTimeSeconds;
	}

	public static double getTimeSinceTeleopInitSeconds() {
		if (Robot.ROBOT_TYPE.isReal()) {
			return GamePeriodUtils.TELEOP_DURATION_SECONDS - DriverStation.getMatchTime();
		}
		if (RobotManager.getTeleopStartTimeSeconds() == -1) {
			return -1;
		}
		return TimeUtil.getCurrentTimeSeconds() - RobotManager.getTeleopStartTimeSeconds();
	}

	public static double getAutonomousStartTimeSeconds() {
		return autonomousStartTimeSeconds;
	}

	public static void setAutonomousStartTimeSeconds(double autonomousStartTimeSeconds) {
		TimeUtil.autonomousStartTimeSeconds = autonomousStartTimeSeconds;
	}

}
