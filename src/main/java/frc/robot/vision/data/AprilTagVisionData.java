package frc.robot.vision.data;

import edu.wpi.first.math.geometry.Pose3d;

public class AprilTagVisionData extends VisionData {

	private final double[] standardDeviations;
	private final double aprilTagHeightMeters;
	private final double distanceFromAprilTagMeters;
	private final int aprilTagId;

	public AprilTagVisionData(
		String sourceName,
		Pose3d estimatedRobotPosition,
		double timestamp,
		double[] standardDeviations,
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

	public double[] getStandardDeviations() {
		return standardDeviations;
	}

	public double getAprilTagHeightMeters() {
		return aprilTagHeightMeters;
	}

	public double getDistanceFromAprilTagMeters() {
		return distanceFromAprilTagMeters;
	}

	public int getTrackedAprilTagId() {
		return aprilTagId;
	}

}
