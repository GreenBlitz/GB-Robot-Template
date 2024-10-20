package frc.robot.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;

public class Tolerances {

	public static final Rotation2d HEADING_TOLERANCE = Rotation2d.fromDegrees(2.5);
	public static final Rotation2d ROTATION_VELOCITY_TOLERANCE = Rotation2d.fromRadians(0.1);
	public static final double TRANSLATION_TOLERANCE_METERS = 0.2;
	public static final double TRANSLATION_VELOCITY_TOLERANCE = 0.15;
	public static final Rotation2d FLYWHEEL_VELOCITY_TOLERANCE = Rotation2d.fromRotations(4);
	public static final double ELEVATOR_POSITION_METERS_TOLERANCE = 0.05;

}
