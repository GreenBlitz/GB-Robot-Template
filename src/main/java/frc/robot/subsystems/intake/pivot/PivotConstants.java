package frc.robot.subsystems.intake.pivot;

import edu.wpi.first.math.geometry.Rotation2d;

public class PivotConstants {

	public static final String LOG_PATH = "Subsystems/Pivot/";

	public static final Rotation2d MAXIMUM_ACHIEVABLE_ANGLE = Rotation2d.fromDegrees(105.6);

	public static final Rotation2d FORWARD_SOFT_LIMIT = Rotation2d.fromDegrees(104);

	public static final Rotation2d REVERSE_SOFT_LIMIT = Rotation2d.fromDegrees(-0.5);

	public static final double GEAR_RATIO = 1 / ((7.0 / 1) * (5.0 / 1) * (60.0 / 16));

	public static final int MEDIAN_FILTER_SIZE = 5;

}
