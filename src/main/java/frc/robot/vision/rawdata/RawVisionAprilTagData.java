package frc.robot.vision.rawdata;

import edu.wpi.first.math.geometry.Pose3d;

public class RawVisionAprilTagData extends RawVisionData {

	private final double aprilTagHeight;
	private final double distanceFromAprilTag;

	public RawVisionAprilTagData(Pose3d estimatedPose, double aprilTagHeight, double distanceFromAprilTag, double timestamp) {
		super(estimatedPose, timestamp);
		this.aprilTagHeight = aprilTagHeight;
		this.distanceFromAprilTag = distanceFromAprilTag;
	}

	public double getAprilTagHeight() {
		return aprilTagHeight;
	}

	public double getDistanceFromAprilTag() {
		return distanceFromAprilTag;
	}

}
