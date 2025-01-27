package frc.robot.autonomous;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;


public class AutonomousConstants {

	public static final String LOG_PATH_PREFIX = "Autonomous";

	public static final PathConstraints REAL_TIME_CONSTRAINTS = new PathConstraints(2.5, 2.5, 4, 4);

	public static final double PATHFINDING_DEADBAND_METERS = 0.5;

	public static final double DISTANCE_FROM_TARGET_TOLERANCE_METERS = 0.02;

	public static final Rotation2d TARGET_ANGLE_TOLERANCE = Rotation2d.fromDegrees(1.5);

	public static final RobotConfig SYNCOPA_ROBOT_CONFIG = new RobotConfig(
		60,
		8.6,
		new ModuleConfig(0.048, 5.24, 0.96, DCMotor.getFalcon500Foc(1), 60, 1),
		0.577
	);

}
