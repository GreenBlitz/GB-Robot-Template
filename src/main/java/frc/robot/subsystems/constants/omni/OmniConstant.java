package frc.robot.subsystems.constants.omni;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotConstants;
import frc.robot.subsystems.constants.turret.Constants;

public class OmniConstant {
	public static final String LOG_PATH = RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Omni";
	public static final String DIGITAL_INPUT_NAME = "NAME???";
	public static final double GEAR_RATIO = 10;
	public static final int CURRENT_LIMIT = 40;
	public static final double MOMENT_OF_INERTIA = 40;
	public static final double DEBOUNCE_TIME = 40;
	public static final boolean IS_FORWARD_LIMIT_SWITCH = true;
	public static final boolean IS_FORWARD_LIMIT_SWITCH_INVERTED = false;
}
