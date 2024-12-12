package frc.robot.poseestimator;


import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.poseestimator.helpers.StandardDeviations2d;

public class PoseEstimatorConstants {

	protected static final String LOG_PATH = "PoseEstimator/";

	public static final int VISION_OBSERVATION_COUNT_FOR_AVERAGED_POSE_CALCULATION = 25;

	protected static final double POSE_BUFFER_SIZE_SECONDS = 2.0;

	public static final AprilTagFields APRIL_TAG_FIELD = AprilTagFields.k2024Crescendo;

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
