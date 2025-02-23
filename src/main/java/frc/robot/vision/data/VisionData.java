package frc.robot.vision.data;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.vision.sources.VisionSource;

public class VisionData {

	private final VisionSource<? extends VisionData> source;
	private final Pose3d estimatedPose;
	private final double timestamp;

	public VisionData(VisionSource<? extends VisionData> source, Pose3d estimatedPose, double timestamp) {
		this.source = source;
		this.estimatedPose = estimatedPose;
		this.timestamp = timestamp;
	}

	public VisionSource<? extends VisionData> getSource() {
		return source;
	}

	public Pose3d getEstimatedPose() {
		return estimatedPose;
	}

	public double getTimestamp() {
		return timestamp;
	}

}
