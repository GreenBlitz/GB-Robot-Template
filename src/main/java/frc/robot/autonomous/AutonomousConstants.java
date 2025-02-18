package frc.robot.autonomous;

import com.pathplanner.lib.path.PathConstraints;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.factories.constants.RealSwerveConstants;


public class AutonomousConstants {

	public static final String LOG_PATH_PREFIX = "Autonomous";

	public static final double PATHFINDING_DEADBAND_METERS = 0.5;

	public static PathConstraints getRealTimeConstraints(Swerve swerve) {
		return new PathConstraints(
			swerve.getConstants().velocityAt12VoltsMetersPerSecond(),
			RealSwerveConstants.ACCELERATION_AT_12_VOLTS_METERS_PER_SECOND_SQUARED,
			swerve.getConstants().maxRotationalVelocityPerSecond().getRadians(),
			RealSwerveConstants.MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND
		);
	}

}
