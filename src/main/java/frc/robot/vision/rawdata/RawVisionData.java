package frc.robot.vision.rawdata;

import edu.wpi.first.math.geometry.Pose3d;

public class RawVisionData {

	protected final Pose3d estimatedPose;
	protected final double timestamp;

	public RawVisionData(Pose3d estimatedPose, double timestamp) {
		this.estimatedPose = estimatedPose;
		this.timestamp = timestamp;
	}

	public Pose3d estimatedPose() {
		return estimatedPose;
	}

	public double timestamp() {
		return timestamp;
	}

}
