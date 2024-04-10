package frc.robot.constants;

import frc.utils.RobotTypeUtils;

public class RobotConstants {

    public static final RobotTypeUtils.RobotType ROBOT_TYPE =
            RobotTypeUtils.determineRobotType(RobotTypeUtils.RobotType.REAL);
    public static final String USB_LOG_PATH = "/dev/sda1";
    public static final String SAFE_ROBORIO_LOG_PATH = "/home/lvuser/logs";
    public static final double BATTERY_VOLTAGE = 12;
}
