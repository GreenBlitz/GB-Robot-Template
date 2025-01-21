package frc.robot.autonomous;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;


public class AutonomousConstants {

	public static final String LOG_PATH_PREFIX = "Autonomous/";

	public static final PathConstraints REAL_TIME_CONSTRAINTS = new PathConstraints(2.5, 2.5, 4, 4);

	public static final double PATHFINDING_DEADBAND_METERS = 0.5;

	public static final double DISTANCE_FROM_TARGET_TO_START_NEXT_COMMAND_METERS = 0.6;

	public static final double DISTANCE_FROM_TARGET_TOLERANCE_METERS = 0.02;

	public static final Rotation2d TARGET_ANGLE_TOLERANCE = Rotation2d.fromDegrees(1.5);

	public static final RobotConfig SYNCOPA_ROBOT_CONFIG = new RobotConfig(
		60,
		8.6,
		new ModuleConfig(0.048, 5.24, 0.96, DCMotor.getFalcon500Foc(1), 60, 1),
		0.577
	);

	public static class LinkedWaypoints {

		public static final Pair<String, Translation2d> AL1 = Pair.of("AL1", new Translation2d()); //todo real values for all

		public static final Pair<String, Translation2d> AL2 = Pair.of("AL2", new Translation2d(7.6, 6.17));

		public static final Pair<String, Translation2d> AL3 = Pair.of("AL3", new Translation2d());

		public static final Pair<String, Translation2d> AL4 = Pair.of("AL4", new Translation2d(7.6, 4.02));

		public static final Pair<String, Translation2d> AL5 = Pair.of("AL5", new Translation2d());

		public static final Pair<String, Translation2d> AL6 = Pair.of("AL6", new Translation2d(7.6, 1.9));

		public static final Pair<String, Translation2d> AL7 = Pair.of("AL7", new Translation2d());

		public static final Pair<String, Translation2d> US = Pair.of("US", new Translation2d(1.15, 7.05));

		public static final Pair<String, Translation2d> LS = Pair.of("LS", new Translation2d(1.15, 0.97));

		public static final Pair<String, Translation2d> A = Pair.of("A", new Translation2d());

		public static final Pair<String, Translation2d> B = Pair.of("B", new Translation2d());

		public static final Pair<String, Translation2d> C = Pair.of("C", new Translation2d(3.72, 3.02));

		public static final Pair<String, Translation2d> D = Pair.of("D", new Translation2d());

		public static final Pair<String, Translation2d> E = Pair.of("E", new Translation2d());

		public static final Pair<String, Translation2d> F = Pair.of("F", new Translation2d(5.26, 3.02));

		public static final Pair<String, Translation2d> G = Pair.of("G", new Translation2d());

		public static final Pair<String, Translation2d> H = Pair.of("H", new Translation2d(5.75, 4.19));

		public static final Pair<String, Translation2d> I = Pair.of("I", new Translation2d(5.25, 5.04));

		public static final Pair<String, Translation2d> J = Pair.of("J", new Translation2d());

		public static final Pair<String, Translation2d> K = Pair.of("K", new Translation2d());

		public static final Pair<String, Translation2d> L = Pair.of("L", new Translation2d(3.72, 5.04));

	}

}
