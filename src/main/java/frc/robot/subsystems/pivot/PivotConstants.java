package frc.robot.subsystems.pivot;

import edu.wpi.first.math.geometry.Rotation2d;

public class PivotConstants {

	public static final String LOG_PATH = "Subsystems/Pivot";

	public static final Rotation2d FORWARD_ANGLE_LIMIT = Rotation2d.fromDegrees(67);
	public static final Rotation2d BACKWARD_ANGLE_LIMIT = Rotation2d.fromDegrees(25);

	public static final Rotation2d MINIMUM_ACHIEVABLE_ANGLE = Rotation2d.fromDegrees(17);

	public static final int MEDIAN_FILTER_SIZE = 10;

}
