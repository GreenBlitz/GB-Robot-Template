package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;

public class RobotHeadingEstimatorConstants {

	public static final double POSE_BUFFER_SIZE_SECONDS = 2.0;

	public static final double DEFAULT_GYRO_STANDARD_DEVIATION = 0.0001;

	public static final double DEFAULT_VISION_STANDARD_DEVIATION = 0.03;

	public static final int ANGLE_ACCUMULATOR_SIZE = 30;

	public static final Rotation2d VISION_HEADING_AVERAGE_COMPARISON_TOLERANCE = Rotation2d.fromDegrees(5);

}
