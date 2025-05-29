package frc.robot.subsystems.climb.lifter;

import edu.wpi.first.math.geometry.Rotation2d;

public class LifterConstants {

	public static final Rotation2d MINIMUM_ACHIEVABLE_POSITION = Rotation2d.fromDegrees(-10);
	public static final Rotation2d MINIMUM_CLIMB_POSITION = Rotation2d.fromDegrees(-4);
	public static final Rotation2d CLIMB_OFFSET_AFTER_LIMIT_SWITCH = Rotation2d.fromDegrees(15);
	public static final Rotation2d RESET_OFFSET = Rotation2d.fromDegrees(0.5);
	public static final double LIFTER_LENGTH_METERS = 0.3;
	public static final double GEAR_RATIO = 70.0 / (4 / 7.0) * (4 * 5);

}
