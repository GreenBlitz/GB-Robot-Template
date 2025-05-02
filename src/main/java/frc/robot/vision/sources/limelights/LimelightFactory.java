package frc.robot.vision.sources.limelights;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.vision.data.AprilTagVisionData;
import frc.robot.vision.sources.IndpendentHeadingVisionSource;
import frc.robot.vision.sources.RobotHeadingRequiringVisionSource;
import frc.utils.Filter;

public final class LimelightFactory {

	public static RobotHeadingRequiringVisionSource createRobotHeadingRequiringLimelight(
		String cameraNetworkTablesName,
		String parentLogPath,
		String sourceName,
		Filter<? super AprilTagVisionData> filter,
		Pose3d cameraPoseOffset,
		boolean regulateTemperature
	) {
		return new LimeLightSource(
			cameraNetworkTablesName,
			parentLogPath,
			sourceName,
			filter,
			cameraPoseOffset,
			LimelightPoseEstimationMethod.MEGATAG_2,
			regulateTemperature
		);
	}

	public static IndpendentHeadingVisionSource createRobotHeadingEstimatingLimelight(
		String cameraNetworkTablesName,
		String parentLogPath,
		String sourceName,
		Filter<? super AprilTagVisionData> filter,
		Pose3d cameraPoseOffset,
		boolean regulateTemperature
	) {
		return new LimeLightSource(
			cameraNetworkTablesName,
			parentLogPath,
			sourceName,
			filter,
			cameraPoseOffset,
			LimelightPoseEstimationMethod.MEGATAG_1,
			regulateTemperature
		);
	}

}
