package frc.robot.poseestimator.WPILibPoseEstimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.poseestimator.helpers.StandardDeviations2D;
import frc.robot.vision.data.AprilTagVisionData;

import java.util.function.Function;

public class WPILibPoseEstimatorConstants {

	public static final String WPILIB_POSEESTIMATOR_LOGPATH = "WPILibPoseEstimator/";

	//@formatter:off
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
	//formatter:on

	public static final double VISION_STDEVS_FACTOR = 0.1;


	public static final Rotation2d INITIAL_GYRO_ANGLE = new Rotation2d();

	public static final Pose2d STARTING_ODOMETRY_POSE = new Pose2d();

	public static final Function<AprilTagVisionData, StandardDeviations2D> VISION_STDDEVS_TRANSFORM = aprilTagVisionData -> new StandardDeviations2D(aprilTagVisionData.getDistanceFromAprilTagMeters() * WPILibPoseEstimatorConstants.VISION_STDEVS_FACTOR);


}
