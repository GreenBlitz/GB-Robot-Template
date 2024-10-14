package frc.robot.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;

public class Tolerances {

	public static final Rotation2d SWERVE_HEADING = Rotation2d.fromDegrees(3);
	public static final Rotation2d ROTATION_VELOCITY_DEADBAND = Rotation2d.fromRadians(0.2);
	public static final double TRANSLATION_METERS = 0.05;
	public static final double TRANSLATION_VELOCITY_DEADBAND = 0.05;

	public static final Rotation2d ELBOW_POSITION = Rotation2d.fromDegrees(1.5);
	public static final Rotation2d ELBOW_POSITION_TRANSFER = Rotation2d.fromDegrees(3);
	public static final Rotation2d WRIST_POSITION = Rotation2d.fromDegrees(5);
	public static final Rotation2d PIVOT_POSITION = Rotation2d.fromDegrees(1);
	public static final Rotation2d FLYWHEEL_VELOCITY_PER_SECOND = Rotation2d.fromRotations(2);

	public static final Rotation2d ROLLER_INTAKE_ROTATIONS = Rotation2d.fromRotations(0.5);

}
