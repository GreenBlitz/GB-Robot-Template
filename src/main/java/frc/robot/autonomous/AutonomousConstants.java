package frc.robot.autonomous;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;


public class AutonomousConstants {

	public static final String LOG_PATH_PREFIX = "Autonomous";

	public static final PathConstraints REAL_TIME_CONSTRAINTS = new PathConstraints(2.5, 2.5, 4, 4);

	public static final double PATHFINDING_DEADBAND_METERS = 0.5;

	public static final Pose2d TARGET_POSE_TOLERANCES = new Pose2d(0.02, 0.02, Rotation2d.fromDegrees(2));

	public static final RobotConfig ROBOT_CONFIG = new RobotConfig(
			60,
			8.6,
			new ModuleConfig(0.0508, 4.477, 0.96, DCMotor.getKrakenX60Foc(1), 7.13, 60, 1),
			new Translation2d(0.3, 0.3),
			new Translation2d(0.3, -0.3),
			new Translation2d(-0.3, 0.3),
			new Translation2d(-0.3, -0.3)
	);

	public static class LinkedWaypoints {

		public static final Pair<String, Pose2d> AUTO_LINE1 = Pair.of("AL1", new Pose2d(7.6, 7.26, Rotation2d.fromDegrees(180)));

		public static final Pair<String, Pose2d> AUTO_LINE2 = Pair.of("AL2", new Pose2d(7.6, 6.17, Rotation2d.fromDegrees(180)));

		public static final Pair<String, Pose2d> AUTO_LINE3 = Pair.of("AL3", new Pose2d(7.6, 5.07, Rotation2d.fromDegrees(180)));

		public static final Pair<String, Pose2d> AUTO_LINE4 = Pair.of("AL4", new Pose2d(7.6, 4.02, Rotation2d.fromDegrees(180)));

		public static final Pair<String, Pose2d> AUTO_LINE5 = Pair.of("AL5", new Pose2d(7.6, 3, Rotation2d.fromDegrees(180)));

		public static final Pair<String, Pose2d> AUTO_LINE6 = Pair.of("AL6", new Pose2d(7.6, 1.9, Rotation2d.fromDegrees(180)));

		public static final Pair<String, Pose2d> AUTO_LINE7 = Pair.of("AL7", new Pose2d(7.6, 0.81, Rotation2d.fromDegrees(180)));

		public static final Pair<String, Pose2d> UPPER_CORAL_STATION = Pair.of("US", new Pose2d(1.15, 7.05, Rotation2d.fromDegrees(-54)));

		public static final Pair<String, Pose2d> LOWER_CORAL_STATION = Pair.of("LS", new Pose2d(1.15, 0.97, Rotation2d.fromDegrees(54)));

		public static final Pair<String, Pose2d> A = Pair.of("A", new Pose2d(3.22, 4.19, Rotation2d.fromDegrees(0)));

		public static final Pair<String, Pose2d> B = Pair.of("B", new Pose2d(3.22, 3.86, Rotation2d.fromDegrees(0)));

		public static final Pair<String, Pose2d> C = Pair.of("C", new Pose2d(3.72, 3.02, Rotation2d.fromDegrees(60)));

		public static final Pair<String, Pose2d> D = Pair.of("D", new Pose2d(4, 2.86, Rotation2d.fromDegrees(60)));

		public static final Pair<String, Pose2d> E = Pair.of("E", new Pose2d(4.97, 2.86, Rotation2d.fromDegrees(120)));

		public static final Pair<String, Pose2d> F = Pair.of("F", new Pose2d(5.26, 3.02, Rotation2d.fromDegrees(120)));

		public static final Pair<String, Pose2d> G = Pair.of("G", new Pose2d(5.75, 3.86, Rotation2d.fromDegrees(180)));

		public static final Pair<String, Pose2d> H = Pair.of("H", new Pose2d(5.75, 4.19, Rotation2d.fromDegrees(180)));

		public static final Pair<String, Pose2d> I = Pair.of("I", new Pose2d(5.25, 5.04, Rotation2d.fromDegrees(-120)));

		public static final Pair<String, Pose2d> J = Pair.of("J", new Pose2d(4.97, 5.19, Rotation2d.fromDegrees(-120)));

		public static final Pair<String, Pose2d> K = Pair.of("K", new Pose2d(4, 5.19, Rotation2d.fromDegrees(-60)));

		public static final Pair<String, Pose2d> L = Pair.of("L", new Pose2d(3.72, 5.04, Rotation2d.fromDegrees(-60)));

	}

}
