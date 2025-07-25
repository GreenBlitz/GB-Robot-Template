package frc.robot.autonomous;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.constants.field.Field;
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

	public static final Pose2d NET_AUTO_RELEASE_DEADBANDS = new Pose2d(1, 1, Rotation2d.fromRadians(2));

	public static final Translation2d DEFAULT_LEFT_FLOOR_ALGAE_POSITION = Field.getAllianceRelative(new Translation2d(5.860, 7.2), true, true);
	public static final Translation2d DEFAULT_RIGHT_FLOOR_ALGAE_POSITION = Field.getAllianceRelative(new Translation2d(6.700, 7.2), true, true);

	public static final double DEFAULT_AUTO_DRIVE_POWER = -0.3;

	public static final double DEFAULT_AUTO_DRIVE_TIME_SECONDS = 1;

	public static final double INTAKING_TIMEOUT_SECONDS = 4;

	public static final double BACK_OFF_FROM_REEF_DISTANCE_METERS = -1;
	public static final double DISTANCE_FROM_ALGAE_FOR_FLOOR_INTAKE = 0.1;

	public static final double FIRST_ALGAE_REMOVE_TIMEOUT_SECONDS = 1;
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

}
