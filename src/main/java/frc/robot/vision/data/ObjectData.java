package frc.robot.vision.data;

import edu.wpi.first.math.geometry.Translation2d;

public class ObjectData {

	private final Translation2d robotRelativeEstimatedPose;
	private final String objectType;
	private final double timestamp;

	public ObjectData(Translation2d robotRelativeEstimatedPose, String objectType, double timestamp) {
		this.robotRelativeEstimatedPose = robotRelativeEstimatedPose;
		this.objectType = objectType;
		this.timestamp = timestamp;
	}

	public Translation2d getRobotRelativeEstimatedPose() {
		return robotRelativeEstimatedPose;
	}

	public String getObjectType() {
		return objectType;
	}

	public double getTimestamp() {
		return timestamp;
	}

}
