package frc.robot.poseestimator;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.poseestimator.linearfilters.LinearFilterType;

public class PoseEstimatorConstants {

	protected static final String LOG_PATH = "PoseEstimator/";

	static class VISION_LINEAR_FILTER {

		protected static final String LOG_PATH = "VisionLinearFilter/";

		protected static final LinearFilterType FILTER_TYPE = LinearFilterType.GBLinearFilter;

		protected static final int SAMPLE_COUNT = 10;

	}

	protected static final int OBSERVATION_COUNT_FOR_POSE_CALIBRATION = 45;

	public static final double POSE_BUFFER_SIZE_SECONDS = 2.0;

	public static final int ODOMETRY_FREQUENCY_HERTZ = 250;

	public static final Pose2d INITIAL_ROOT_POSE = new Pose2d(2, 5, new Rotation2d());

	public static final double[] ODOMETRY_STANDARD_DEVIATIONS = {0.003, 0.003, 0.0002};

}
