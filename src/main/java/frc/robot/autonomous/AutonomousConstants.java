package frc.robot.autonomous;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.subsystems.swerve.factories.constants.RealSwerveConstants;


public class AutonomousConstants {

	public static final String LOG_PATH_PREFIX = "Autonomous";

	public static final PathConstraints REAL_TIME_CONSTRAINTS = new PathConstraints(
		RealSwerveConstants.VELOCITY_AT_12_VOLTS_METERS_PER_SECOND,
		RealSwerveConstants.ACCELERATION_AT_12_VOLTS_METERS_PER_SECOND_SQUARED,
		RealSwerveConstants.MAX_ROTATIONAL_VELOCITY_PER_SECOND.getRadians(),
		4
	);

	public static final double PATHFINDING_DEADBAND_METERS = 0.5;

	public static final Pose2d TARGET_POSE_TOLERANCES = new Pose2d(0.02, 0.02, Rotation2d.fromDegrees(2));

	public static final RobotConfig ROBOT_CONFIG = new RobotConfig(
		50,
		4.6875,
		new ModuleConfig(0.0508, 4.477, 0.96, DCMotor.getKrakenX60Foc(1), 7.13, 60, 1),
		new Translation2d(0.3, 0.3),
		new Translation2d(0.3, -0.3),
		new Translation2d(-0.3, 0.3),
		new Translation2d(-0.3, -0.3)
	);

	public static class LinkedWaypoints {

		public static final Pair<String, Pose2d> AUTO_LINE1 = Pair.of("AL1", new Pose2d(7.58, 7.26, Rotation2d.fromDegrees(180)));

		public static final Pair<String, Pose2d> AUTO_LINE2 = Pair.of("AL2", new Pose2d(7.58, 6.17, Rotation2d.fromDegrees(180)));

		public static final Pair<String, Pose2d> AUTO_LINE3 = Pair.of("AL3", new Pose2d(7.58, 5.07, Rotation2d.fromDegrees(180)));

		public static final Pair<String, Pose2d> AUTO_LINE4 = Pair.of("AL4", new Pose2d(7.58, 4.02, Rotation2d.fromDegrees(180)));

		public static final Pair<String, Pose2d> AUTO_LINE5 = Pair.of("AL5", new Pose2d(7.58, 3, Rotation2d.fromDegrees(180)));

		public static final Pair<String, Pose2d> AUTO_LINE6 = Pair.of("AL6", new Pose2d(7.58, 1.9, Rotation2d.fromDegrees(180)));

		public static final Pair<String, Pose2d> AUTO_LINE7 = Pair.of("AL7", new Pose2d(7.58, 0.81, Rotation2d.fromDegrees(180)));

		public static final Pair<String, Pose2d> UPPER_CORAL_STATION = Pair.of("US", new Pose2d(1.63, 7.34, Rotation2d.fromDegrees(-54)));

		public static final Pair<String, Pose2d> LOWER_CORAL_STATION = Pair.of("LS", new Pose2d(1.61, 0.7, Rotation2d.fromDegrees(54)));

		public static final Pair<String, Pose2d> A = Pair.of("A", new Pose2d(3.16, 4.19, Rotation2d.fromDegrees(0)));

		public static final Pair<String, Pose2d> B = Pair.of("B", new Pose2d(3.16, 3.86, Rotation2d.fromDegrees(0)));

		public static final Pair<String, Pose2d> C = Pair.of("C", new Pose2d(3.65, 2.96, Rotation2d.fromDegrees(60)));

		public static final Pair<String, Pose2d> D = Pair.of("D", new Pose2d(3.97, 2.8, Rotation2d.fromDegrees(60)));

		public static final Pair<String, Pose2d> E = Pair.of("E", new Pose2d(5.01, 2.79, Rotation2d.fromDegrees(120)));

		public static final Pair<String, Pose2d> F = Pair.of("F", new Pose2d(5.3, 2.94, Rotation2d.fromDegrees(120)));

		public static final Pair<String, Pose2d> G = Pair.of("G", new Pose2d(5.83, 3.86, Rotation2d.fromDegrees(180)));

		public static final Pair<String, Pose2d> H = Pair.of("H", new Pose2d(5.83, 4.19, Rotation2d.fromDegrees(180)));

		public static final Pair<String, Pose2d> I = Pair.of("I", new Pose2d(5.3, 5.08, Rotation2d.fromDegrees(-120)));

		public static final Pair<String, Pose2d> J = Pair.of("J", new Pose2d(5.02, 5.25, Rotation2d.fromDegrees(-120)));

		public static final Pair<String, Pose2d> K = Pair.of("K", new Pose2d(3.95, 5.26, Rotation2d.fromDegrees(-60)));

		public static final Pair<String, Pose2d> L = Pair.of("L", new Pose2d(3.68, 5.1, Rotation2d.fromDegrees(-60)));

	}

}
