package frc.robot.autonomous;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.constants.field.enums.Branch;
import frc.constants.field.enums.ReefSide;
import frc.robot.scoringhelpers.ScoringHelpers;
import frc.robot.statemachine.StateMachineConstants;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.factories.constants.RealSwerveConstants;


public class AutonomousConstants {

	public static final String LOG_PATH_PREFIX = "Autonomous";

	public static final double PATHFINDING_DEADBAND_METERS = 0.5;

	public static final Pose2d TARGET_POSE_TOLERANCES = new Pose2d(0.035, 0.035, Rotation2d.fromDegrees(2));

	public static final double DEFAULT_AUTO_DRIVE_POWER = -0.3;

	public static final double DEFAULT_AUTO_DRIVE_TIME_SECONDS = 1;

	public static final double INTAKING_TIMEOUT_SECONDS = 4;

	public static final double BACK_OFF_FROM_REEF_DISTANCE_METERS = -1;

	public static final double FIRST_ALGAE_REMOVE_TIMEOUT_SECONDS = 0.9;
	public static final double ALGAE_REMOVE_TIMEOUT_SECONDS = 0.5;

	public static PathConstraints getRealTimeConstraints(Swerve swerve) {
		return new PathConstraints(
			swerve.getConstants().velocityAt12VoltsMetersPerSecond(),
			2.5, // RealSwerveConstants.ACCELERATION_AT_12_VOLTS_METERS_PER_SECOND_SQUARED,
			swerve.getConstants().maxRotationalVelocityPerSecond().getRadians(),
			RealSwerveConstants.MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND
		);
	}

	public static PathConstraints getRealTimeConstraintsForAuto(Swerve swerve) {
		return new PathConstraints(
			swerve.getConstants().velocityAt12VoltsMetersPerSecond(),
			3.5, // RealSwerveConstants.ACCELERATION_AT_12_VOLTS_METERS_PER_SECOND_SQUARED,
			swerve.getConstants().maxRotationalVelocityPerSecond().getRadians(),
			RealSwerveConstants.MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND
		);
	}

	public static class LinkedWaypoints {

		public static final Pair<String, Pose2d> AUTO_LINE_1 = Pair.of("AL1", new Pose2d(7.58, 7.26, Rotation2d.fromDegrees(-137)));

		public static final Pair<String, Pose2d> AUTO_LINE_2 = Pair.of("AL2", new Pose2d(7.58, 6.17, Rotation2d.fromDegrees(-145)));

		public static final Pair<String, Pose2d> AUTO_LINE_3 = Pair.of("AL3", new Pose2d(7.58, 5.07, Rotation2d.fromDegrees(-157)));

		public static final Pair<String, Pose2d> AUTO_LINE_4 = Pair.of("AL4", new Pose2d(7.58, 4.02, Rotation2d.fromDegrees(180)));

		public static final Pair<String, Pose2d> AUTO_LINE_5 = Pair.of("AL5", new Pose2d(7.58, 3, Rotation2d.fromDegrees(157)));

		public static final Pair<String, Pose2d> AUTO_LINE_6 = Pair.of("AL6", new Pose2d(7.58, 1.9, Rotation2d.fromDegrees(152)));

		public static final Pair<String, Pose2d> AUTO_LINE_7 = Pair.of("AL7", new Pose2d(7.58, 0.81, Rotation2d.fromDegrees(137)));

		public static final Pair<String, Pose2d> UPPER_CORAL_STATION_2 = Pair.of("US2", new Pose2d(1.63, 7.34, Rotation2d.fromDegrees(-54)));

		public static final Pair<String, Pose2d> LOWER_CORAL_STATION_2 = Pair.of("LS2", new Pose2d(1.61, 0.7, Rotation2d.fromDegrees(54)));

		public static final Pair<String, Pose2d> A = Pair.of("A", getRobotBranchScoringBluePose(Branch.A));

		public static final Pair<String, Pose2d> B = Pair.of("B", getRobotBranchScoringBluePose(Branch.B));

		public static final Pair<String, Pose2d> C = Pair.of("C", getRobotBranchScoringBluePose(Branch.C));

		public static final Pair<String, Pose2d> D = Pair.of("D", getRobotBranchScoringBluePose(Branch.D));

		public static final Pair<String, Pose2d> E = Pair.of("E", getRobotBranchScoringBluePose(Branch.E));

		public static final Pair<String, Pose2d> F = Pair.of("F", getRobotBranchScoringBluePose(Branch.F));

		public static final Pair<String, Pose2d> G = Pair.of("G", getRobotBranchScoringBluePose(Branch.G));

		public static final Pair<String, Pose2d> H = Pair.of("H", getRobotBranchScoringBluePose(Branch.H));

		public static final Pair<String, Pose2d> I = Pair.of("I", getRobotBranchScoringBluePose(Branch.I));

		public static final Pair<String, Pose2d> J = Pair.of("J", getRobotBranchScoringBluePose(Branch.J));

		public static final Pair<String, Pose2d> K = Pair.of("K", getRobotBranchScoringBluePose(Branch.K));

		public static final Pair<String, Pose2d> L = Pair.of("L", getRobotBranchScoringBluePose(Branch.L));

		public static final Pair<String, Pose2d> ALGAE_REMOVE_C = Pair.of("ARC", ScoringHelpers.getAlgaeRemovePose(ReefSide.C));

		public static final Pair<String, Pose2d> ALGAE_REMOVE_D = Pair.of("ARD", ScoringHelpers.getAlgaeRemovePose(ReefSide.D));

		public static final Pair<String, Pose2d> ALGAE_REMOVE_E = Pair.of("ARE", ScoringHelpers.getAlgaeRemovePose(ReefSide.E));

		public static final Pair<String, Pose2d> LEFT_NET = Pair.of("LN", new Pose2d(7.578, 6.740, Rotation2d.fromDegrees(0)));

		public static final Pair<String, Pose2d> MIDDLE_NET = Pair.of("MN", new Pose2d(7.578, 6.045, Rotation2d.fromDegrees(0)));

		public static final Pair<String, Pose2d> RIGHT_NET = Pair.of("RN", new Pose2d(7.578, 5.064, Rotation2d.fromDegrees(0)));

		private static Pose2d getRobotBranchScoringBluePose(Branch branch) {
			return ScoringHelpers.getRobotBranchScoringPose(branch, StateMachineConstants.ROBOT_SCORING_DISTANCE_FROM_REEF_METERS, false);
		}

	}

}
