package frc.robot.vision.data;

import edu.wpi.first.math.geometry.Translation2d;

public class ObjectData {

	private final Translation2d robotRelativeEstimatedTranslation;
	private final String objectType;
	private final double timestamp;

	public ObjectData(Translation2d robotRelativeEstimatedTranslation, String objectType, double timestamp) {
		this.robotRelativeEstimatedTranslation = robotRelativeEstimatedTranslation;
		this.objectType = objectType;
		this.timestamp = timestamp;
	}

	public Translation2d getRobotRelativeEstimatedTranslation() {
		return robotRelativeEstimatedTranslation;
	}

	public String getObjectType() {
		return objectType;
	}

	public double getTimestamp() {
		return timestamp;
	}

}
