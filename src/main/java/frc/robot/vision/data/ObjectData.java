package frc.robot.vision.data;

import edu.wpi.first.math.geometry.Pose3d;

public class ObjectData extends VisionData {

	private final String objectType;

	public ObjectData(String sourceName, Pose3d estimatedPose, String objectType, double timestamp) {
		super(sourceName, estimatedPose, timestamp);
		this.objectType = objectType;
	}

	public String getObjectType() {
		return objectType;
	}

}
