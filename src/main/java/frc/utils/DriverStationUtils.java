package frc.utils;

import edu.wpi.first.wpilibj.DriverStation;

public class DriverStationUtils {

    private static final DriverStation.Alliance DEFAULT_ALLIANCE = DriverStation.Alliance.Blue;

    public static boolean isBlueAlliance() {
        return getAlliance().equals(DriverStation.Alliance.Blue);
    }

    public static DriverStation.Alliance getAlliance() {
        return DriverStation.getAlliance().orElse(DEFAULT_ALLIANCE);
    }

    public static boolean isRedAlliance() {
        return getAlliance().equals(DriverStation.Alliance.Red);
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

}
