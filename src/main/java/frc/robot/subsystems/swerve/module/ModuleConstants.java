package frc.robot.subsystems.swerve.module;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.utils.battery.BatteryUtils;

public class ModuleConstants {

	public static final double VOLTAGE_COMPENSATION_SATURATION = BatteryUtils.DEFAULT_VOLTAGE;
	public static final boolean DEFAULT_IS_CLOSE_LOOP = true;
	public static final String LOG_PATH_ADDITION = "Modules/";

	static final Rotation2d CALIBRATION_MODULE_ANGLE_TOLERANCE = Rotation2d.fromDegrees(3);
	static final Rotation2d CALIBRATION_MODULE_ANGLE_VELOCITY_PER_SECOND_DEADBAND = Rotation2d.fromDegrees(3);

}
