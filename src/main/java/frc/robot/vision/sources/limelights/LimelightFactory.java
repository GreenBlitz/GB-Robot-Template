package frc.robot.vision.sources.limelights;

import frc.robot.vision.data.AprilTagVisionData;
import frc.robot.vision.sources.IndpendentHeadingVisionSource;
import frc.robot.vision.sources.RobotHeadingRequiringVisionSource;
import frc.robot.vision.sources.VisionSource;
import frc.utils.Filter;

public class LimelightFactory {

	public static VisionSource<AprilTagVisionData> createIndependentRobotPoseEstimatingLimelight(
		String name,
		String parentLogPath,
		Filter<AprilTagVisionData> filter
	) {
		return new LimeLightSource(name, parentLogPath, filter, LimelightPoseEstimationMethod.MEGATAG_1);
	}

	public static RobotHeadingRequiringVisionSource createRobotHeadingRequiringLimelight(
		String name,
		String parentLogPath,
		Filter<AprilTagVisionData> filter
	) {
		return new LimeLightSource(name, parentLogPath, filter, LimelightPoseEstimationMethod.MEGATAG_2);
	}

	public static IndpendentHeadingVisionSource createRobotHeadingEstimatingLimelight(
		String name,
		String parentLogPath,
		Filter<AprilTagVisionData> filter
	) {
		return new LimeLightSource(name, parentLogPath, filter, LimelightPoseEstimationMethod.MEGATAG_1);
	}

}
