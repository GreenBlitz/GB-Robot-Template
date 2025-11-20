package frc.robot.subsystems.constants.omni;

import frc.robot.RobotConstants;

public class OmniConstant {

	public static final String LOG_PATH = RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Omni";
	public static final String FUNNEL_INPUT_NAME = "FunnelSensor";
	public static final double GEAR_RATIO = 25.0 / 6.0;
	public static final int CURRENT_LIMIT = 20;
	public static final double MOMENT_OF_INERTIA = 0.001;
	public static final double DEBOUNCE_TIME = 0.01;
	public static final boolean IS_FORWARD_LIMIT_SWITCH = false;
	public static final boolean IS_FORWARD_LIMIT_SWITCH_INVERTED = false;
	public static final boolean IS_INVERTED = true;

}
