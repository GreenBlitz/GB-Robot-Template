package frc.robot.autonomous;

import com.pathplanner.lib.path.PathConstraints;
import frc.robot.subsystems.swerve.factories.constants.RealSwerveConstants;


public class AutonomousConstants {

	public static final String LOG_PATH_PREFIX = "Autonomous";

	public static final PathConstraints REAL_TIME_CONSTRAINTS = new PathConstraints(
		RealSwerveConstants.VELOCITY_AT_12_VOLTS_METERS_PER_SECOND,
		RealSwerveConstants.ACCELERATION_AT_12_VOLTS_METERS_PER_SECOND_SQUARED,
		RealSwerveConstants.MAX_ROTATIONAL_VELOCITY_PER_SECOND.getRadians(),
		4
	);

	public static final double CLOSE_TO_TARGET_POSITION_DEADBAND_METERS = 0.5;

}
