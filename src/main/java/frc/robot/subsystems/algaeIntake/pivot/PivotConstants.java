package frc.robot.subsystems.algaeIntake.pivot;

import edu.wpi.first.math.geometry.Rotation2d;

public class PivotConstants {

	public static final Rotation2d TOLERANCE = Rotation2d.fromDegrees(3);
	public static final double LENGTH_METERS = 0.359;
	public static final Rotation2d MIN_POSITION = Rotation2d.fromDegrees(0);
	public static final Rotation2d MAX_POSITION = Rotation2d.fromDegrees(120);
	public static final Rotation2d STARTING_POSITION = Rotation2d.fromDegrees(110);
	public static final boolean IS_INVERTED = false;

	public static final Rotation2d FORWARD_LIMIT = Rotation2d.fromDegrees(120);
	public static final Rotation2d BACKWARD_LIMIT = Rotation2d.fromDegrees(0);


}
