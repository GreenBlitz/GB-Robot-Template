package frc.robot.vision.rawdata;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;

public class RawAprilTagVisionData extends RawVisionData {

	private final double aprilTagHeight;
	private final double distanceFromAprilTag;
	private final AprilTag aprilTag;

	public RawAprilTagVisionData(
		Pose3d estimatedPose,
		double timestamp,
		boolean isDataValid,
		double aprilTagHeight,
		double distanceFromAprilTag,
		AprilTag aprilTag
	) {
		super(estimatedPose, timestamp, isDataValid);
		this.aprilTagHeight = aprilTagHeight;
		this.distanceFromAprilTag = distanceFromAprilTag;
		this.aprilTag = aprilTag;
	}

	public double getAprilTagHeight() {
		return aprilTagHeight;
	}

	public double getDistanceFromAprilTag() {
		return distanceFromAprilTag;
	}

	public AprilTag getTrackedAprilTag() {
		return aprilTag;
	}

}
