package frc.robot.autonomous;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
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

	public static class LinkedWaypoints {

		public static final Pair<String, Pose2d> AUTO_LINE1 = Pair.of("AL1", new Pose2d()); // todo real values for all

		public static final Pair<String, Pose2d> AUTO_LINE2 = Pair.of("AL2", new Pose2d(7.6, 6.17, Rotation2d.fromDegrees(180)));

		public static final Pair<String, Pose2d> AUTO_LINE3 = Pair.of("AL3", new Pose2d());

		public static final Pair<String, Pose2d> AUTO_LINE4 = Pair.of("AL4", new Pose2d(7.6, 4.02, Rotation2d.fromDegrees(180)));

		public static final Pair<String, Pose2d> AUTO_LINE5 = Pair.of("AL5", new Pose2d());

		public static final Pair<String, Pose2d> AUTO_LINE6 = Pair.of("AL6", new Pose2d(7.6, 1.9, Rotation2d.fromDegrees(180)));

		public static final Pair<String, Pose2d> AUTO_LINE7 = Pair.of("AL7", new Pose2d());

		public static final Pair<String, Pose2d> UPPER_CORAL_STATION = Pair.of("US", new Pose2d(1.15, 7.05, Rotation2d.fromDegrees(-54)));

		public static final Pair<String, Pose2d> LOWER_CORAL_STATION = Pair.of("LS", new Pose2d(1.15, 0.97, Rotation2d.fromDegrees(54)));

		public static final Pair<String, Pose2d> A = Pair.of("A", new Pose2d());

		public static final Pair<String, Pose2d> B = Pair.of("B", new Pose2d());

		public static final Pair<String, Pose2d> C = Pair.of("C", new Pose2d(3.72, 3.02, Rotation2d.fromDegrees(60)));

		public static final Pair<String, Pose2d> D = Pair.of("D", new Pose2d());

		public static final Pair<String, Pose2d> E = Pair.of("E", new Pose2d());

		public static final Pair<String, Pose2d> F = Pair.of("F", new Pose2d(5.26, 3.02, Rotation2d.fromDegrees(120)));

		public static final Pair<String, Pose2d> G = Pair.of("G", new Pose2d());

		public static final Pair<String, Pose2d> H = Pair.of("H", new Pose2d(5.75, 4.19, Rotation2d.fromDegrees(180)));

		public static final Pair<String, Pose2d> I = Pair.of("I", new Pose2d(5.25, 5.04, Rotation2d.fromDegrees(-120)));

		public static final Pair<String, Pose2d> J = Pair.of("J", new Pose2d());

		public static final Pair<String, Pose2d> K = Pair.of("K", new Pose2d());

		public static final Pair<String, Pose2d> L = Pair.of("L", new Pose2d(3.72, 5.04, Rotation2d.fromDegrees(-60)));

	}

}
