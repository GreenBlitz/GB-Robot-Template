package frc.robot.vision.data;

import edu.wpi.first.math.geometry.Pose3d;

public class VisionData {

	private final String sourceName;
	private final Pose3d estimatedPose;
	private final double timestamp;

	public VisionData(String sourceName, Pose3d estimatedPose, double timestamp) {
		this.sourceName = sourceName;
		this.estimatedPose = estimatedPose;
		this.timestamp = timestamp;
	}

	public final String getSourceName() {
		return sourceName;
	}

	public final Pose3d getEstimatedPose() {
		return estimatedPose;
	}

	public final double getTimestamp() {
		return timestamp;
	}

}
