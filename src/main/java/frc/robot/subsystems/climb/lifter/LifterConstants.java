package frc.robot.subsystems.climb.lifter;

import edu.wpi.first.math.geometry.Rotation2d;

public class LifterConstants {

	public static final Rotation2d MINIMUM_ACHIEVABLE_POSITION = Rotation2d.fromDegrees(-10);
	public static final Rotation2d RESET_OFFSET = Rotation2d.fromDegrees(0.5);
	public static final double LIFTER_LENGTH_METERS = 0.3;
	public static final double GEAR_RATIO = 70.0 / (4 / 7.0) * (4 * 5);

	public static final int REVERSE_LIMIT_SWITCH_CHANNEL = 2;
	public static final double REVERSE_LIMIT_SWITCH_DEBOUNCE_TIME = 0.04;


}
