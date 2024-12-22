package frc.robot.vision.data;

import edu.wpi.first.math.geometry.Pose3d;

public class VisionData {

	private final Pose3d estimatedPose;
	private final double timestamp;

	public VisionData(Pose3d estimatedPose, double timestamp) {
		this.estimatedPose = estimatedPose;
		this.timestamp = timestamp;
	}

	public Pose3d getEstimatedPose() {
		return estimatedPose;
	}

	public double getTimestamp() {
		return timestamp;
	}

}
