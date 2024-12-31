package frc.robot.poseestimator.WPILibPoseEstimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.poseestimator.helpers.StandardDeviations2D;

public class WPILibPoseEstimatorConstants {

	public static final int POSE_BUFFER_SIZE_SECONDS = 2;

	public static final StandardDeviations2D DEFAULT_ODOMETRY_STANDARD_DEVIATIONS = new StandardDeviations2D(
		0.003,
		0.003,
		0.003
	);
	public static final StandardDeviations2D DEFAULT_VISION_STANDARD_DEVIATIONS = new StandardDeviations2D(
		0.0003,
		0.0003,
		0.003
	);

	public static final Rotation2d STARTING_ODOMETRY_ANGLE = new Rotation2d();
	public static final Pose2d STARTING_ODOMETRY_POSE = new Pose2d();

	public static final String WPILIB_POSEESTIMATOR_LOGPATH = "WPILibPoseEstimator/";

}
