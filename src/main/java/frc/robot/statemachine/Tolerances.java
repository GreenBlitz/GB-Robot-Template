package frc.robot.statemachine;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.constants.field.Field;

public class Tolerances {

	public static final double ELEVATOR_HEIGHT_METERS = 0.01;

	public static final Rotation2d ARM_POSITION = Rotation2d.fromDegrees(1.5);

	public static final Pose2d SCORING_POSITION = new Pose2d(0.05, 0.05, Rotation2d.fromDegrees(3));
	public static final Pose2d SCORING_DEADBANDS = new Pose2d(0.1, 0.1, Rotation2d.fromRadians(0.1));

	public static final Pose2d L1_SCORING_POSITION = new Pose2d(Field.REEF_SIDE_LENGTH_METERS / 2.0, 0.05, Rotation2d.fromDegrees(3));
	public static final Pose2d L1_SCORING_DEADBANDS = new Pose2d(1, 0.1, Rotation2d.fromRadians(0.1));

}
