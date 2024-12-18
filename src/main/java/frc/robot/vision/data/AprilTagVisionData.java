package frc.robot.vision.data;

import edu.wpi.first.math.geometry.Pose3d;

public class AprilTagVisionData extends VisionData {

	private final double aprilTagHeightMeters;
	private final double distanceFromAprilTagMeters;
	private final int aprilTagId;

	public AprilTagVisionData(Pose3d estimatedPose, double timestamp, double aprilTagHeightMeters, double distanceFromAprilTagMeters, int aprilTagId) {
		super(estimatedPose, timestamp);
		this.aprilTagHeightMeters = aprilTagHeightMeters;
		this.distanceFromAprilTagMeters = distanceFromAprilTagMeters;
		this.aprilTagId = aprilTagId;
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
