package frc.robot.vision.rawdata;

import edu.wpi.first.math.geometry.Pose3d;

public class RawVisionData {

	private final Pose3d estimatedPose;
	private final double timestamp;
	private final boolean isDataValid;

	public RawVisionData(Pose3d estimatedPose, double timestamp, boolean isDataValid) {
		this.estimatedPose = estimatedPose;
		this.timestamp = timestamp;
		this.isDataValid = isDataValid;
	}

	public Pose3d getEstimatedPose() {
		return estimatedPose;
	}

	public double getTimestamp() {
		return timestamp;
	}

	public boolean getIsDataValid() {
		return isDataValid;
	}

}
