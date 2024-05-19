package frc.utils;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.ToggleConstants;

public class DriverStationUtils {

    private static final DriverStation.Alliance DEFAULT_ALLIANCE = DriverStation.Alliance.Red;

    public static DriverStation.Alliance getAlliance() {
        if (RobotConstants.ROBOT_TYPE.isSimulation()) {
            return ToggleConstants.SIMULATION_ALLIANCE.get();
        }
        return DriverStation.getAlliance().orElse(DEFAULT_ALLIANCE);
    }

    public static boolean isBlueAlliance() {
        return getAlliance().equals(DriverStation.Alliance.Blue);
    }

    public static boolean isRedAlliance() {
        return !isBlueAlliance();
    }

    public static boolean isConnectedToFMS() {
        return DriverStation.isFMSAttached();
    }

    public static boolean isAutonomous() {
        return DriverStation.isAutonomous();
    }

    public static boolean isAutonomousEnabled() {
        return DriverStation.isAutonomousEnabled();
    }

    public static boolean isTeleop() {
        return DriverStation.isTeleop();
    }

    public static boolean isTeleopEnabled() {
        return DriverStation.isTeleopEnabled();
    }

    public static boolean isTest() {
        return DriverStation.isTest();
    }

    public static boolean isTestEnabled() {
        return DriverStation.isTestEnabled();
    }

    public static boolean isGame() {
        return DriverStation.getMatchType() != DriverStation.MatchType.None;
    }

}
