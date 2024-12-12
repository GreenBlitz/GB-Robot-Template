package frc.robot.vision.rawdata;

import edu.wpi.first.math.geometry.Pose3d;

public class RawVisionData implements IRawVisionData {

	private final Pose3d estimatedPose;
	private final double timestamp;

	public RawVisionData(Pose3d estimatedPose, double timestamp) {
		this.estimatedPose = estimatedPose;
		this.timestamp = timestamp;
	}

	@Override
	public Pose3d getEstimatedPose() {
		return estimatedPose;
	}

	@Override
	public double getTimestamp() {
		return timestamp;
	}

}
