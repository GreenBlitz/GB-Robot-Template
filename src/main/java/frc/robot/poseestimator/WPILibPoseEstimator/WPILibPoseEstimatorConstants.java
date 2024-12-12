package frc.robot.poseestimator.WPILibPoseEstimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class WPILibPoseEstimatorConstants {

	public static final StandardDeviations2d DEFAULT_ODOMETRY_STANDARD_DEVIATIONS = new StandardDeviations2d(
		0.0003,
		0.00003,
		Rotation2d.fromRadians(0.003)
	);
	public static final StandardDeviations2d DEFAULT_VISION_STANDARD_DEVIATIONS = new StandardDeviations2d(
		0.0003,
		0.00003,
		Rotation2d.fromRadians(0.003)
	);

	public static final Rotation2d STARTING_ODOMETRY_ANGLE = new Rotation2d();
	public static final Pose2d STARTING_ODOMETRY_POSE = new Pose2d();
}
