package frc.robot.autonomous;

import com.pathplanner.lib.path.PathConstraints;


public class AutonomousConstants {

	public static final String LOG_PATH_PREFIX = "Autonomous";

	public static final PathConstraints REAL_TIME_CONSTRAINTS = new PathConstraints(2.5, 2.5, 4, 4);

	public static final double CLOSE_TO_TARGET_POSITION_DEADBAND_METERS = 0.5;

}
