package frc.robot.vision.rawdata;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.poseestimator.observations.IRobotPoseVisionObservation;

public class RawAprilTagVisionData extends RawVisionData implements IRobotPoseVisionObservation {

	private final double aprilTagHeight;
	private final double distanceFromAprilTag;
	private final AprilTag aprilTag;

	public RawAprilTagVisionData(Pose3d estimatedPose, double aprilTagHeight, double distanceFromAprilTag, double timestamp, AprilTag aprilTag) {
		super(estimatedPose, timestamp);
		this.aprilTagHeight = aprilTagHeight;
		this.distanceFromAprilTag = distanceFromAprilTag;
		this.aprilTag = aprilTag;
	}

	@Override
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
