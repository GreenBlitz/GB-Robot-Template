package frc.robot.structures;

import edu.wpi.first.math.geometry.Rotation2d;

public class Tolerances {

	public static final Rotation2d SWERVE_HEADING = Rotation2d.fromDegrees(1.5);
	public static final Rotation2d ROTATION_VELOCITY_DEADBAND = Rotation2d.fromRadians(0.05);
	public static final double TRANSLATION_METERS = 0.05;
	public static final double TRANSLATION_VELOCITY_DEADBAND = 0.05;


	public static final double SPEED_TOLERANCE_METERS_PER_SECOND = 0.1;

}
