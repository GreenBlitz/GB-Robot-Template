package frc.robot.poseestimator.helpers.RobotHeadingEstimator;

public class RobotHeadingEstimatorConstants {

	public static final String DEFAULT_HEADING_ESTIMATOR_LOGPATH = "HeadingEstimator/";

	public static final String VISION_NOISE_STANDARD_DEVIATION_LOGPATH_ADDITION = "VisionNoiseStandardDeviation/";

	public static final String ESTIMATED_HEADING_LOGPATH_ADDITION = "EstimatedHeading/";

	public static final String ESTIMATED_HEADING_DIFFERENCE_FROM_GYRO_YAW_LOGPATH_ADDITION = "EstimatedHeadingDifferenceFromGyroYaw/";

	public static final String VISION_HEADING_INPUT_LOGPATH_ADDITION = "VisionHeadingInput/";


	public static final double POSE_BUFFER_SIZE_SECONDS = 2.0;

	public static final int AMOUNT_OF_SOURCE_TYPES = 2;

	public static final double DEFAULT_GYRO_STANDARD_DEVIATION = 0.0001;

	public static final double DEFAULT_VISION_STANDARD_DEVIATION = 0.001;

	public static final double MAXIMUM_STANDARD_DEVIATION_TOLERANCE = 0.0025;

	public static final int ESTIMATION_GYRO_PAIR_BUFFER_SIZE = 50;

}
