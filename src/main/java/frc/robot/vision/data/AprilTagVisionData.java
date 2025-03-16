package frc.robot.vision.data;

import edu.wpi.first.math.geometry.Pose3d;
import frc.utils.math.StandardDeviations3D;

import java.util.Optional;

public class AprilTagVisionData extends VisionData {

	protected StandardDeviations3D standardDeviations;
	protected double aprilTagHeightMeters;
	protected double distanceFromAprilTagMeters;
	protected int aprilTagId;

	public AprilTagVisionData(
		String sourceName,
		Pose3d estimatedRobotPosition,
		double timestamp,
		StandardDeviations3D standardDeviations,
		double aprilTagHeightMeters,
		double distanceFromAprilTagMeters,
		int aprilTagId
	) {
		super(sourceName, estimatedRobotPosition, timestamp);
		this.standardDeviations = standardDeviations;
		this.aprilTagHeightMeters = aprilTagHeightMeters;
		this.distanceFromAprilTagMeters = distanceFromAprilTagMeters;
		this.aprilTagId = aprilTagId;
	}

	public void setValues(
		String sourceName,
		Pose3d estimatedRobotPosition,
		double timestamp,
		StandardDeviations3D standardDeviations,
		double aprilTagHeightMeters,
		double distanceFromAprilTagMeters,
		int aprilTagId
	) {
		setValues(sourceName, estimatedRobotPosition, timestamp);
		this.standardDeviations = standardDeviations;
		this.aprilTagHeightMeters = aprilTagHeightMeters;
		this.distanceFromAprilTagMeters = distanceFromAprilTagMeters;
		this.aprilTagId = aprilTagId;
	}

	public StandardDeviations3D getStandardDeviations() {
		return standardDeviations;
	}

	public final double getAprilTagHeightMeters() {
		return aprilTagHeightMeters;
	}

	public final double getDistanceFromAprilTagMeters() {
		return distanceFromAprilTagMeters;
	}

	public final int getTrackedAprilTagId() {
		return aprilTagId;
	}

	public static AprilTagVisionData updateInstanceOf(
		Optional<AprilTagVisionData> instance,
		String sourceName,
		Pose3d estimatedRobotPosition,
		double timestamp,
		StandardDeviations3D standardDeviations,
		double aprilTagHeightMeters,
		double distanceFromAprilTagMeters,
		int aprilTagId
	) {
		if (instance.isEmpty()) {
			return new AprilTagVisionData(
				sourceName,
				estimatedRobotPosition,
				timestamp,
				standardDeviations,
				aprilTagHeightMeters,
				distanceFromAprilTagMeters,
				aprilTagId
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
				aprilTagId
			);
		return instance.get();
	}

}
