package frc.robot.autonomous;

import com.pathplanner.lib.path.PathConstraints;
import frc.robot.subsystems.swerve.Swerve;


public class AutonomousConstants {

	public static final String LOG_PATH_PREFIX = "Autonomous";

	public static final double PATHFINDING_DEADBAND_METERS = 0.5;

	public static final double DEFAULT_AUTO_DRIVE_POWER = 0;

	public static final double DEFAULT_AUTO_DRIVE_TIME_SECONDS = 0;

	public static PathConstraints getRealTimeConstraints(Swerve swerve) {
		return new PathConstraints(
			swerve.getConstants().velocityAt12VoltsMetersPerSecond(),
			2.5,
			swerve.getConstants().maxRotationalVelocityPerSecond().getRadians(),
			4
		);
	}

}
