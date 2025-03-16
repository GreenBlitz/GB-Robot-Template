package frc.robot.vision.data;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.vision.sources.limelights.LimelightPoseEstimationMethod;
import frc.utils.math.StandardDeviations3D;

import java.util.Optional;

public final class LimeLightAprilTagVisionData extends AprilTagVisionData {

	private LimelightPoseEstimationMethod poseEstimationMethod;

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

	public void setValues(
		String sourceName,
		Pose3d estimatedRobotPosition,
		double timestamp,
		StandardDeviations3D standardDeviations,
		double aprilTagHeightMeters,
		double distanceFromAprilTagMeters,
		int aprilTagId,
		LimelightPoseEstimationMethod poseEstimationMethod
	) {
		setValues(
			sourceName,
			estimatedRobotPosition,
			timestamp,
			standardDeviations,
			aprilTagHeightMeters,
			distanceFromAprilTagMeters,
			aprilTagId
		);
		this.poseEstimationMethod = poseEstimationMethod;
	}

	public LimelightPoseEstimationMethod getPoseEstimationMethod() {
		return poseEstimationMethod;
	}

	public static LimeLightAprilTagVisionData updateInstanceOf(
		Optional<LimeLightAprilTagVisionData> instance,
		String sourceName,
		Pose3d estimatedRobotPosition,
		double timestamp,
		StandardDeviations3D standardDeviations,
		double aprilTagHeightMeters,
		double distanceFromAprilTagMeters,
		int aprilTagId,
		LimelightPoseEstimationMethod poseEstimationMethod
	) {
		if (instance.isEmpty()) {
			return new LimeLightAprilTagVisionData(
				sourceName,
				estimatedRobotPosition,
				timestamp,
				standardDeviations,
				aprilTagHeightMeters,
				distanceFromAprilTagMeters,
				aprilTagId,
				poseEstimationMethod
			);
		}
		instance.get()
			.setValues(
				sourceName,
				estimatedRobotPosition,
				timestamp,
				standardDeviations,
				aprilTagHeightMeters,
				distanceFromAprilTagMeters,
				aprilTagId,
				poseEstimationMethod
			);
		return instance.get();
	}

}
