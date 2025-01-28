package frc.robot.autonomous;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Rotation2d;


public class AutonomousConstants {

	public static final String LOG_PATH_PREFIX = "Autonomous";

	public static final PathConstraints REAL_TIME_CONSTRAINTS = new PathConstraints(2.5, 2.5, 4, 4);

	public static final double PATHFINDING_DEADBAND_METERS = 0.5;

	public static final double DISTANCE_FROM_TARGET_TOLERANCE_METERS = 0.02;

	public static final Rotation2d TARGET_ANGLE_TOLERANCE = Rotation2d.fromDegrees(1.5);

}
