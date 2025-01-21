package frc.robot.vision.sources.limelights;

import frc.robot.vision.data.AprilTagVisionData;
import frc.robot.vision.sources.IndpendentHeadingVisionSource;
import frc.robot.vision.sources.RobotHeadingRequiringVisionSource;
import frc.utils.Filter;

public class LimelightFactory {

	public static RobotHeadingRequiringVisionSource createRobotHeadingRequiringLimelight(
		String name,
		String parentLogPath,
		String sourceLogPathAddition,
		Filter<AprilTagVisionData> filter
	) {
		return new LimeLightSource(name, parentLogPath, sourceLogPathAddition, filter, LimelightPoseEstimationMethod.MEGATAG_2);
	}

	public static IndpendentHeadingVisionSource createRobotHeadingEstimatingLimelight(
		String name,
		String parentLogPath,
		String sourceLogPathAddition,
		Filter<AprilTagVisionData> filter
	) {
		return new LimeLightSource(name, parentLogPath, sourceLogPathAddition, filter, LimelightPoseEstimationMethod.MEGATAG_1);
	}

}
