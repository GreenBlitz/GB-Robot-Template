package frc.robot.statemachine;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Tolerances {

	public static final double ELEVATOR_HEIGHT_METERS = 0.05;

	public static final Rotation2d ARM_INTERPOLATION_POSITION = Rotation2d.fromDegrees(0.5);
	public static final Rotation2d ARM_POSITION = Rotation2d.fromDegrees(1.5);
	public static final Rotation2d ARM_ALGAE_TRANSFER_POSITION = Rotation2d.fromDegrees(5);
	public static final Rotation2d ALGAE_RELEASE_ARM_POSITION = Rotation2d.fromDegrees(10);

	public static final Pose2d NET_OPENING_SUPERSTRUCTURE_POSITION_METERS = new Pose2d(0.07, 0.07, Rotation2d.fromDegrees(2));
	public static final Pose2d NET_SCORING_POSITION_METERS = new Pose2d(0.07, 0.07, Rotation2d.fromDegrees(10));

	public static final Rotation2d PIVOT = Rotation2d.fromDegrees(3);

}
