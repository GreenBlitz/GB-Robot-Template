package frc.robot.constants;

import frc.utils.RobotTypeUtils;
import frc.utils.RobotTypeUtils.RobotType;

public class RobotConstants {

    public static final RobotType ROBOT_TYPE = RobotTypeUtils.determineRobotType(RobotType.REAL);
    public static final String USB_LOG_PATH = "/dev/sda1";
    public static final String SAFE_ROBORIO_LOG_PATH = "/home/lvuser/logs";
}
