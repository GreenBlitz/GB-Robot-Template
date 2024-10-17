package frc.robot.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;

public class Tolerances {

	public static final Rotation2d HEADING_TOLERANCE = Rotation2d.fromDegrees(1.5);
	public static final Rotation2d ROTATION_VELOCITY_TOLERANCE = Rotation2d.fromRadians(0.05);
	public static final double TRANSLATION_TOLERANCE_METERS = 0.05;
	public static final double TRANSLATION_VELOCITY_TOLERANCE = 0.05;
	public static final Rotation2d FLYWHEEL_VELOCITY_TOLERANCE = Rotation2d.fromRotations(1);
	public static final double ELEVATOR_POSITION_METERS_TOLERANCE = 0.02;

	public static final double OUT_OF_ROBOT_NOTE_DETECTION_TIMEOUT_SECONDS = 0.2;

}
