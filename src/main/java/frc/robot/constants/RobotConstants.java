package frc.robot.constants;

import edu.wpi.first.wpilibj.DriverStation;
import frc.utils.RobotTypeUtils;
import frc.utils.RobotTypeUtils.RobotType;

public class RobotConstants {

    public static final RobotType ROBOT_TYPE = RobotTypeUtils.determineRobotType(RobotType.REAL);

    public static final DriverStation.Alliance SIMULATION_ALLIANCE = DriverStation.Alliance.Blue;

    public static final boolean IS_RUNNING_ON_USB = false;

    public static final boolean ENABLE_KEYBOARD = true;

    public static final boolean ENABLE_BATTERY_LIMITER = true;

}
