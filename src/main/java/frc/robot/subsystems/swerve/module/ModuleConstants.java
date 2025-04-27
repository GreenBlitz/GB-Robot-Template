package frc.robot.subsystems.swerve.module;

import frc.utils.battery.BatteryUtil;

public class ModuleConstants {

	public static final double VOLTAGE_COMPENSATION_SATURATION = BatteryUtil.DEFAULT_VOLTAGE;
	public static final boolean DEFAULT_IS_CLOSE_LOOP = true;
	public static final boolean IS_CURRENT_CONTROL = true;
	public static final String MODULES_LOG_PATH_ADDITION = "/Modules";

}
