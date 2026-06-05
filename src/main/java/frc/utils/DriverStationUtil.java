package frc.utils;

import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchType;
import org.wpilib.driverstation.internal.DriverStationBackend;

public class DriverStationUtil {

	private static final Alliance DEFAULT_ALLIANCE = Alliance.RED;

	public static Alliance getAlliance() {
		return DriverStationBackend.getAlliance().orElse(DEFAULT_ALLIANCE);
	}

	public static boolean isBlueAlliance() {
		return getAlliance().equals(Alliance.BLUE);
	}

	public static boolean isRedAlliance() {
		return !isBlueAlliance();
	}

	public static boolean isConnectedToFMS() {
		return DriverStationBackend.isFMSAttached();
	}

	public static boolean isAutonomous() {
		return DriverStationBackend.isAutonomous();
	}

	public static boolean isAutonomousEnabled() {
		return DriverStationBackend.isAutonomousEnabled();
	}

	public static boolean isTeleop() {
		return DriverStationBackend.isTeleop();
	}

	public static boolean isTeleopEnabled() {
		return DriverStationBackend.isTeleopEnabled();
	}

	public static boolean isUtility() {
		return DriverStationBackend.isUtility();
	}

	public static boolean isUtilityEnabled() {
		return DriverStationBackend.isUtilityEnabled();
	}

	public static boolean isDisabled() {
		return DriverStationBackend.isDisabled();
	}

	public static boolean isMatch() {
		return DriverStationBackend.getMatchType() != MatchType.NONE;
	}

}
