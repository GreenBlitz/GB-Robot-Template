package frc.robot.vision.data;

import edu.wpi.first.math.geometry.Pose3d;

public class ObjectData {

	private final Pose3d estimatedPose;
	private final String objectType;
	private final double timestamp;


	public ObjectData(Pose3d estimatedPose, String objectType, double timestamp) {
		this.estimatedPose = estimatedPose;
		this.objectType = objectType;
		this.timestamp = timestamp;
	}
	public Pose3d getEstimatedPose() {
		return estimatedPose;
	}

	public String getObjectType() {
		return objectType;
	}

	public double getTimestamp() {
		return timestamp;
	}

}
