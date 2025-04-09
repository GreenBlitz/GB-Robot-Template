package frc.robot.vision.data;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.vision.sources.limelights.LimelightPoseEstimationMethod;
import frc.utils.math.StandardDeviations3D;

public final class LimeLightAprilTagVisionData extends AprilTagVisionData {

	private final LimelightPoseEstimationMethod poseEstimationMethod;

	public LimeLightAprilTagVisionData(
		String sourceName,
		Pose3d estimatedRobotPosition,
		double timestamp,
		StandardDeviations3D standardDeviations,
		double aprilTagHeightMeters,
		double distanceFromAprilTagMeters,
		int aprilTagId,
		LimelightPoseEstimationMethod poseEstimationMethod
	) {
		super(sourceName, estimatedRobotPosition, timestamp, standardDeviations, aprilTagHeightMeters, distanceFromAprilTagMeters, aprilTagId);
		this.poseEstimationMethod = poseEstimationMethod;
	}

	public final LimelightPoseEstimationMethod getPoseEstimationMethod() {
		return poseEstimationMethod;
	}

}
