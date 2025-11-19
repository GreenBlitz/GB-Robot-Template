package frc.robot.poseestimator.WPILibPoseEstimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.utils.math.StandardDeviations2D;


public class WPILibPoseEstimatorConstants {

	public static final String WPILIB_POSEESTIMATOR_LOGPATH = "WPILibPoseEstimator";

	public static final StandardDeviations2D DEFAULT_ODOMETRY_STANDARD_DEVIATIONS = new StandardDeviations2D(0.003, 0.003, 0.003);

	public static final StandardDeviations2D DEFAULT_VISION_STANDARD_DEVIATIONS = new StandardDeviations2D(0.0003, 0.0003, 0.003);

	public static final Rotation2d INITIAL_GYRO_ANGLE = new Rotation2d();

	public static final Pose2d STARTING_ODOMETRY_POSE = new Pose2d();

	public static final int POSE_TO_IMU_ANGLE_DIFFERENCE_BUFFER_SIZE = 50;

	public static double MAX_POSE_TO_IMU_ANGLE_DIFFERENCE_STD_DEV = 0.0025;

	public static double UNOFFSETTED_IMU_ANGLE_BUFFER_SIZE_SECONDS = 2;

}
