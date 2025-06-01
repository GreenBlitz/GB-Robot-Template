package frc.robot.vision.data;

import edu.wpi.first.math.geometry.Translation2d;

public class ObjectData {

	private final Translation2d estimatedPose;
	private final String objectType;
	private final double timestamp;


	public ObjectData(Translation2d estimatedPose, String objectType, double timestamp) {
		this.estimatedPose = estimatedPose;
		this.objectType = objectType;
		this.timestamp = timestamp;
	}

	public Translation2d getEstimatedPose() {
		return estimatedPose;
	}

	public String getObjectType() {
		return objectType;
	}

	public double getTimestamp() {
		return timestamp;
	}

}
