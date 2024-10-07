package frc.robot.poseestimator;


import frc.robot.poseestimator.linearfilters.LinearFilterType;

public class PoseEstimatorConstants {

	protected static final String LOG_PATH = "PoseEstimator/";

	protected static final int VISION_OBSERVATION_COUNT_FOR_AVERAGED_POSE_CALCULATION = 45;

	public static final double POSE_BUFFER_SIZE_SECONDS = 2.0;

	public static final double[] DEFAULT_ODOMETRY_STANDARD_DEVIATIONS = {0.003, 0.003, 0.0002};

	static class LinearFiltersConstants {

		public static final String LOG_PATH = PoseEstimatorConstants.LOG_PATH + "LinearFilters/";

		public static final LinearFilterType FILTER_TYPE = LinearFilterType.GBLinearFilter;

		public static double SAMPLE_COUNT = 10;

	}

}
