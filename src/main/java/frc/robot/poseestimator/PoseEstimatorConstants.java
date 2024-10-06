package frc.robot.poseestimator;


public class PoseEstimatorConstants {

	protected static final String LOG_PATH = "PoseEstimator/";

	protected static final int OBSERVATION_COUNT_FOR_POSE_CALIBRATION = 45;

	public static final double POSE_BUFFER_SIZE_SECONDS = 2.0;

	public static final double[] DEFAULT_ODOMETRY_STANDARD_DEVIATIONS = {0.003, 0.003, 0.0002};

}
