package frc.robot.poseestimator;


import edu.wpi.first.apriltag.AprilTagFields;

public class PoseEstimatorConstants {

	protected static final String LOG_PATH = "PoseEstimator/";

	public static final int VISION_OBSERVATION_COUNT_FOR_AVERAGED_POSE_CALCULATION = 25;

	protected static final double POSE_BUFFER_SIZE_SECONDS = 2.0;

	public static final AprilTagFields APRIL_TAG_FIELD = AprilTagFields.k2024Crescendo;

	public static final double[] DEFAULT_ODOMETRY_STANDARD_DEVIATIONS = {0.003, 0.003, 0.0002};

}
