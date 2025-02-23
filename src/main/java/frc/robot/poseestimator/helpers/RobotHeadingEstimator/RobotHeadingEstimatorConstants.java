package frc.robot.poseestimator.helpers.RobotHeadingEstimator;

import frc.robot.vision.VisionConstants;
import frc.robot.vision.VisionFilters;
import frc.robot.vision.data.VisionData;
import frc.robot.vision.sources.limelights.LimeLightSource;
import frc.robot.vision.sources.limelights.LimelightPoseEstimationMethod;
import frc.utils.Filter;

import java.util.function.Function;

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

	public static final double MAXIMUM_STANDARD_DEVIATION_TOLERANCE = 0.001;

	public static final int ESTIMATION_GYRO_PAIR_BUFFER_SIZE = 50;


	public static final Function<RobotHeadingEstimator, Filter<VisionData>> YAW_FILTER_FOR_HEADING_ESTIMATION = headingEstimator -> new Filter<>(data -> VisionFilters.isYawAtAngle(() -> {
		if (
			data.getSource() instanceof LimeLightSource limelightSource
				&& limelightSource.getPoseEstimationMethod() == LimelightPoseEstimationMethod.MEGATAG_2
		) {
			return headingEstimator.getEstimatedHeading();
		}
		return data.getEstimatedPose().getRotation().toRotation2d();
	}, VisionConstants.YAW_FILTER_TOLERANCE).apply((VisionData) data));
}
