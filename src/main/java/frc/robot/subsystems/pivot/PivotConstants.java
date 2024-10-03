package frc.robot.subsystems.pivot;

import edu.wpi.first.math.geometry.Rotation2d;

public class PivotConstants {

	public static final String LOG_PATH = "Subsystems/Pivot";

	public static final double GEAR_RATIO = 15.0 * (72.0 / 14.0) * 2.0;

	public static final Rotation2d FORWARD_ANGLE_LIMIT = Rotation2d.fromDegrees(60);
	public static final Rotation2d BACKWARD_ANGLE_LIMIT = Rotation2d.fromDegrees(17);

	public static final Rotation2d MINIMUM_ACHIEVABLE_ANGLE = Rotation2d.fromDegrees(16);

	public static final int MEDIAN_FILTER_SIZE = 10;

}
