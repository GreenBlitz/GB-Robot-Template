package frc.robot.vision.data;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.vision.objectdetection.ObjectType;

public class ObjectData {

	private final Translation2d robotRelativeEstimatedTranslation;
	private final ObjectType objectType;
	private final double timestamp;

	public ObjectData(Translation2d robotRelativeEstimatedTranslation, ObjectType objectType, double timestamp) {
		this.robotRelativeEstimatedTranslation = robotRelativeEstimatedTranslation;
		this.objectType = objectType;
		this.timestamp = timestamp;
	}

	public Translation2d getRobotRelativeEstimatedTranslation() {
		return robotRelativeEstimatedTranslation;
	}

	public ObjectType getObjectType() {
		return objectType;
	}

	public double getTimestamp() {
		return timestamp;
	}

}
