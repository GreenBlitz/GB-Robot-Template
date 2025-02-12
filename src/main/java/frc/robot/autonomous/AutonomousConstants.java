package frc.robot.autonomous;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.subsystems.swerve.factories.constants.RealSwerveConstants;
import frc.robot.subsystems.swerve.factories.modules.constants.RealModuleConstants;


public class AutonomousConstants {

	public static final String LOG_PATH_PREFIX = "Autonomous";

	public static final PathConstraints REAL_TIME_CONSTRAINTS = new PathConstraints(
		RealSwerveConstants.VELOCITY_AT_12_VOLTS_METERS_PER_SECOND,
		RealSwerveConstants.ACCELERATION_AT_12_VOLTS_METERS_PER_SECOND_SQUARED,
		RealSwerveConstants.MAX_ROTATIONAL_VELOCITY_PER_SECOND.getRadians(),
		4
	);

	public static final double CLOSE_TO_TARGET_POSITION_DEADBAND_METERS = 0.5;

	public static final RobotConfig ROBOT_CONFIG = new RobotConfig(
		52,
		4.6875,
		new ModuleConfig(
			RealModuleConstants.WHEEL_DIAMETER_METERS / 2,
			RealSwerveConstants.VELOCITY_AT_12_VOLTS_METERS_PER_SECOND,
			0.96,
			DCMotor.getKrakenX60Foc(1),
			7.13,
			60,
			1
		),
		RealModuleConstants.LOCATIONS
	);

}
