package frc.robot.vision.data;

import edu.wpi.first.math.geometry.Pose3d;

public class AprilTagVisionData extends VisionData {

	private final double aprilTagHeight;
	private final double distanceFromAprilTag;
	private final int aprilTagId;

	public AprilTagVisionData(Pose3d estimatedPose, double timestamp, double aprilTagHeight, double distanceFromAprilTag, int aprilTagId) {
		super(estimatedPose, timestamp);
		this.aprilTagHeight = aprilTagHeight;
		this.distanceFromAprilTag = distanceFromAprilTag;
		this.aprilTagId = aprilTagId;
	}

	public double getAprilTagHeight() {
		return aprilTagHeight;
	}

	public double getDistanceFromAprilTag() {
		return distanceFromAprilTag;
	}

	public int getTrackedAprilTagId() {
		return aprilTagId;
	}

}
