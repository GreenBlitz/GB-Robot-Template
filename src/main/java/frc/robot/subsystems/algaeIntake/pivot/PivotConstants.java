package frc.robot.subsystems.algaeIntake.pivot;

import edu.wpi.first.math.geometry.Rotation2d;

public class PivotConstants {

	public static final double LENGTH_METERS = 0.359;
	public static final Rotation2d MIN_POSITION = Rotation2d.fromDegrees(-33);
	public static final Rotation2d MAX_POSITION = Rotation2d.fromDegrees(130);
	public static final Rotation2d STARTING_POSITION = Rotation2d.fromDegrees(130);
	public static final double MASS_KG = 2.5;

	public static final Rotation2d FORWARD_LIMIT = Rotation2d.fromDegrees(120);
	public static final Rotation2d BACKWARD_LIMIT = Rotation2d.fromDegrees(-20);


}
