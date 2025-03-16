package frc.robot.vision.data;

import edu.wpi.first.math.geometry.Pose3d;

import java.util.Optional;

public class VisionData {

	protected String sourceName;
	protected Pose3d estimatedPose;
	protected double timestamp;

	public VisionData(String sourceName, Pose3d estimatedPose, double timestamp) {
		this.sourceName = sourceName;
		this.estimatedPose = estimatedPose;
		this.timestamp = timestamp;
	}

	public void setValues(String sourceName, Pose3d estimatedPose, double timestamp) {
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

	public static VisionData updateInstanceOf(Optional<VisionData> instance, String sourceName, Pose3d estimatedPose, double timestamp) {
		if (instance.isEmpty()) {
			return new VisionData(sourceName, estimatedPose, timestamp);
		}
		instance.get().setValues(sourceName, estimatedPose, timestamp);
		return instance.get();
	}

}
