package frc.utils;

import edu.wpi.first.wpilibj.DriverStation;
import frc.RobotManager;
import frc.robot.Robot;
import frc.utils.driverstation.DriverStationUtil;
import frc.utils.time.TimeUtil;
import org.littletonrobotics.junction.Logger;

public class GamePeriodUtils {

	public static final int TRANSITION_SHIFT_DURATION_SECONDS = 10;
	public static final int ALLIANCE_SHIFT_DURATION_SECONDS = 25;
	public static final int ENDGAME_START_TIME_SECONDS = 110;
	public static final int ENDGAME_DURATION_SECONDS = 30;
	public static final int TELEOP_DURATION_SECONDS = ENDGAME_START_TIME_SECONDS + ENDGAME_DURATION_SECONDS;
	public static final int SECONDS_TO_SCORE_AFTER_GAME_ENDS = 3;
	public static final int ACTIVE_HUB_TIME_AFTER_GAME_ENDS_SECONDS = TELEOP_DURATION_SECONDS + SECONDS_TO_SCORE_AFTER_GAME_ENDS;
	public static final int AUTONOMOUS_DURATION_SECONDS = 20;
	public static final int TELEOP_AUTONOMOUS_TRANSITION_DURATION_SECONDS = 10;
	public static final int COMPLETE_GAME_TIME_SECONDS = AUTONOMOUS_DURATION_SECONDS
		+ TELEOP_AUTONOMOUS_TRANSITION_DURATION_SECONDS
		+ TELEOP_DURATION_SECONDS;

	public static final int ACTIVE_1_END_TIME_SINCE_TELEOP_START = TRANSITION_SHIFT_DURATION_SECONDS + ALLIANCE_SHIFT_DURATION_SECONDS;
	public static final int ACTIVE_2_END_TIME_SINCE_TELEOP_START = ACTIVE_1_END_TIME_SINCE_TELEOP_START + ALLIANCE_SHIFT_DURATION_SECONDS;
	public static final int ACTIVE_3_END_TIME_SINCE_TELEOP_START = ACTIVE_2_END_TIME_SINCE_TELEOP_START + ALLIANCE_SHIFT_DURATION_SECONDS;
	public static final int ACTIVE_4_END_TIME_SINCE_TELEOP_START = ACTIVE_3_END_TIME_SINCE_TELEOP_START + ALLIANCE_SHIFT_DURATION_SECONDS;

	public static boolean isTransitionShift() {
		if (!DriverStationUtil.isTeleop()) {
			return false;
		}
		return TimeUtil.getTimeSinceTeleopInitSeconds() <= TRANSITION_SHIFT_DURATION_SECONDS;
	}

	public static boolean hasGameEnded() {
		return TimeUtil.getTimeSinceTeleopInitSeconds() >= TELEOP_DURATION_SECONDS;
	}

	public static boolean isInEndgame() {
		return TimeUtil.getTimeSinceTeleopInitSeconds() >= ENDGAME_START_TIME_SECONDS
			&& TimeUtil.getTimeSinceTeleopInitSeconds() <= ACTIVE_HUB_TIME_AFTER_GAME_ENDS_SECONDS;
	}

	public static boolean isInActive1() {
		if (HubUtil.isRobotAllianceAutoWinner()) {
			return TimeUtil.getTimeSinceTeleopInitSeconds() > ACTIVE_1_END_TIME_SINCE_TELEOP_START
				&& TimeUtil.getTimeSinceTeleopInitSeconds() < ACTIVE_2_END_TIME_SINCE_TELEOP_START;
		} else {
			return TimeUtil.getTimeSinceTeleopInitSeconds() > TRANSITION_SHIFT_DURATION_SECONDS
				&& TimeUtil.getTimeSinceTeleopInitSeconds() < ACTIVE_1_END_TIME_SINCE_TELEOP_START;
		}
	}

	public static boolean isInActive2() {
		if (HubUtil.isRobotAllianceAutoWinner()) {
			return TimeUtil.getTimeSinceTeleopInitSeconds() > ACTIVE_3_END_TIME_SINCE_TELEOP_START
				&& TimeUtil.getTimeSinceTeleopInitSeconds() < ENDGAME_START_TIME_SECONDS;
		} else {
			return TimeUtil.getTimeSinceTeleopInitSeconds() > ACTIVE_2_END_TIME_SINCE_TELEOP_START
				&& TimeUtil.getTimeSinceTeleopInitSeconds() < ACTIVE_3_END_TIME_SINCE_TELEOP_START;
		}
	}

	private static String getCurrentShift() {
		if (
			TimeUtil.getTimeSinceTeleopInitSeconds() > TRANSITION_SHIFT_DURATION_SECONDS
				&& TimeUtil.getTimeSinceTeleopInitSeconds() < ACTIVE_1_END_TIME_SINCE_TELEOP_START
		) {
			return "Shift 1";
		} else if (
			TimeUtil.getTimeSinceTeleopInitSeconds() > ACTIVE_1_END_TIME_SINCE_TELEOP_START
				&& TimeUtil.getTimeSinceTeleopInitSeconds() < ACTIVE_2_END_TIME_SINCE_TELEOP_START
		) {
			return "Shift 2";
		} else if (
			TimeUtil.getTimeSinceTeleopInitSeconds() > ACTIVE_2_END_TIME_SINCE_TELEOP_START
				&& TimeUtil.getTimeSinceTeleopInitSeconds() < ACTIVE_3_END_TIME_SINCE_TELEOP_START
		) {
			return "Shift 3";
		} else {
			return "Shift 4";
		}
	}

	public static double getTimeUntilTransitionShiftEnds() {
		if (!isTransitionShift()) {
			return -1;
		}
		return TRANSITION_SHIFT_DURATION_SECONDS - TimeUtil.getTimeSinceTeleopInitSeconds();
	}

	public static double getTimeUntilGameEnds() {
		if (Robot.ROBOT_TYPE.isReal()) {
			return DriverStation.getMatchTime();
		}
		if (DriverStationUtil.isAutonomous()) {
			return getTimeUntilAutonomousEnds();
		}
		if (DriverStationUtil.isTeleop())
			return TELEOP_DURATION_SECONDS - TimeUtil.getTimeSinceTeleopInitSeconds();
		return -1;
	}

	public static double getTimeUntilShiftEnds() {
		if (!DriverStationUtil.isTeleop()) {
			return -1;
		}
		return ALLIANCE_SHIFT_DURATION_SECONDS
			- ((TimeUtil.getTimeSinceTeleopInitSeconds() - TRANSITION_SHIFT_DURATION_SECONDS) % GamePeriodUtils.ALLIANCE_SHIFT_DURATION_SECONDS);
	}

	public static double getTimeUntilAutonomousEnds() {
		if (!DriverStationUtil.isAutonomous()) {
			return -1;
		}
		return Robot.ROBOT_TYPE.isReal()
			? DriverStation.getMatchTime()
			: AUTONOMOUS_DURATION_SECONDS - (TimeUtil.getCurrentTimeSeconds() - TimeUtil.getAutonomousStartTimeSeconds());
	}

	public static String getCurrentGamePeriod() {
		if (DriverStation.isAutonomous()) {
			return "Autonomous";
		} else if (isTransitionShift()) {
			return "Transition Shift";
		} else if (isInEndgame()) {
			return "Endgame";
		} else {
			return getCurrentShift();
		}
	}

	public static void log() {
		Logger.recordOutput("TimeSinceTeleopInit", TimeUtil.getTimeSinceTeleopInitSeconds());
		Logger.recordOutput("TeleopStartTimeSeconds", RobotManager.getTeleopStartTimeSeconds());
		Logger.recordOutput("AutonomousStartTimeSeconds", TimeUtil.getAutonomousStartTimeSeconds());
	}

}
