package frc.utils;

import edu.wpi.first.wpilibj.DriverStation;
import frc.utils.driverstation.GameSpecificMessageResponse;
import frc.utils.driverstation.DriverStationUtil;
import frc.utils.alerts.Alert;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import java.util.Optional;

public class HubUtil {

	private static Optional<DriverStation.Alliance> autoWinnerAlliance = getAutoWinningAlliance();
	private static Optional<DriverStation.Alliance> autoLosingAlliance = getAutoLosingAlliance();
	private static final LoggedNetworkBoolean isOurHubActiveTunable = new LoggedNetworkBoolean("Tunable/isOurHubActive", false);

	private static final Alert didNotGetAutoWinningAllianceAlert = new Alert(Alert.AlertType.WARNING, "Didn't get auto winning alliance");
	private static final Alert unknownAutoWinningAllianceAlert = new Alert(Alert.AlertType.WARNING, "Unknown auto winner alliance");

	private static Optional<DriverStation.Alliance> getAutoWinningAlliance() {
		if (!DriverStationUtil.isTeleop()) {
			return Optional.empty();
		}
		String gameData = DriverStation.getGameSpecificMessage();
		if (gameData.isEmpty()) {
			if (DriverStationUtil.isTeleop()) {
				didNotGetAutoWinningAllianceAlert.report();
			}
			return Optional.empty();
		}
		Optional<DriverStation.Alliance> alliance = switch (GameSpecificMessageResponse.fromChar(gameData.charAt(0))) {
			case BLUE -> Optional.of(DriverStation.Alliance.Blue);
			case RED -> Optional.of(DriverStation.Alliance.Red);
			case DEFAULT -> Optional.empty();
		};
		if (alliance.isEmpty()) {
			unknownAutoWinningAllianceAlert.report();
			return Optional.empty();
		}
		return alliance;
	}

	private static Optional<DriverStation.Alliance> getAutoLosingAlliance() {
		if (autoWinnerAlliance.isEmpty()) {
			return Optional.empty();
		}
		return switch (autoWinnerAlliance.get()) {
			case Red -> Optional.of(DriverStation.Alliance.Blue);
			case Blue -> Optional.of(DriverStation.Alliance.Red);
		};
	}

	public static void refreshAlliances() {
		if (autoWinnerAlliance.isEmpty() && DriverStationUtil.isTeleop()) {
			autoWinnerAlliance = getAutoWinningAlliance();
			autoLosingAlliance = getAutoLosingAlliance();
		}
	}

	public static Optional<DriverStation.Alliance> getAutoWinnerAlliance() {
		return autoWinnerAlliance;
	}

	public static Optional<DriverStation.Alliance> getAutoLoserAlliance() {
		return autoLosingAlliance;
	}

	public static boolean isAutoWinnerShift(double timeSinceTeleopInitSeconds) {
		return getShiftsPassed(timeSinceTeleopInitSeconds) % 2 != 0;
	}

	public static int getShiftsPassed(double timeSinceTeleopInitSeconds) {
		if (timeSinceTeleopInitSeconds >= GamePeriodUtils.ENDGAME_START_TIME_SECONDS) {
			return (GamePeriodUtils.ENDGAME_START_TIME_SECONDS - GamePeriodUtils.TRANSITION_SHIFT_DURATION_SECONDS)
				/ GamePeriodUtils.ALLIANCE_SHIFT_DURATION_SECONDS;
		} else {
			return (int) (timeSinceTeleopInitSeconds - GamePeriodUtils.TRANSITION_SHIFT_DURATION_SECONDS)
				/ GamePeriodUtils.ALLIANCE_SHIFT_DURATION_SECONDS;
		}
	}

	public static Optional<DriverStation.Alliance> getActiveHub(double timeSinceTeleopInitSeconds) {
		if (DriverStation.isAutonomous() || GamePeriodUtils.isTransitionShift() || GamePeriodUtils.isInEndgame()) {
			return Optional.of(DriverStationUtil.getAlliance());
		} else if (!DriverStationUtil.isTeleop() || GamePeriodUtils.hasGameEnded()) {
			return Optional.empty();
		}

		return isAutoWinnerShift(timeSinceTeleopInitSeconds) ? autoWinnerAlliance : autoLosingAlliance;
	}

	public static boolean isOurHubActive(double timeSinceTeleopInitSeconds) {
		if (!DriverStation.isFMSAttached()) {
			return isOurHubActiveTunable.get();
		}
		if (getActiveHub(timeSinceTeleopInitSeconds).isEmpty()) {
			return false;
		}
		return getActiveHub(timeSinceTeleopInitSeconds).get().equals(DriverStationUtil.getAlliance());
	}

	public static double timeUntilCurrentShiftEndsSeconds(double timeSinceTeleopInitSeconds) {
		if (DriverStationUtil.isAutonomous()) {
			return GamePeriodUtils.getTimeUntilAutonomousEnds();
		}
		if (timeSinceTeleopInitSeconds >= GamePeriodUtils.ACTIVE_HUB_TIME_AFTER_GAME_ENDS_SECONDS || timeSinceTeleopInitSeconds == -1) {
			return 0;
		}
		if (GamePeriodUtils.isTransitionShift()) {
			return GamePeriodUtils.getTimeUntilTransitionShiftEnds();
		} else if (GamePeriodUtils.isInEndgame()) {
			return GamePeriodUtils.getTimeUntilGameEnds();
		}
		return GamePeriodUtils.getTimeUntilShiftEnds();
	}

	public static double getTimeLeftUntilActiveSeconds(double timeSinceTeleopInitSeconds) {
		if (isOurHubActive(timeSinceTeleopInitSeconds)) {
			return 0;
		}
		return timeUntilCurrentShiftEndsSeconds(timeSinceTeleopInitSeconds);
	}

	public static double getTimeLeftUntilInactiveSeconds(double timeSinceTeleopInitSeconds) {
		if (!isOurHubActive(timeSinceTeleopInitSeconds)) {
			return 0;
		}
		return timeUntilCurrentShiftEndsSeconds(timeSinceTeleopInitSeconds);
	}

	public static boolean isRobotAllianceAutoWinner() {
//		if (autoWinnerAlliance.isEmpty()) {
//			return false;
//		}
//		return DriverStationUtil.getAlliance().equals(autoWinnerAlliance.get());
		return true;
	}

	public static boolean isRobotAllianceAutoWinnerForLog() {
		if (autoWinnerAlliance.isEmpty()) {
			return false;
		}
		return DriverStationUtil.getAlliance().equals(autoWinnerAlliance.get());
	}

}
