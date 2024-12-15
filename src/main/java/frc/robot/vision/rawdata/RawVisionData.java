package frc.robot.vision.rawdata;

import edu.wpi.first.math.geometry.Pose3d;

public class RawVisionData {

	private final Pose3d estimatedPose;
	private final double timestamp;
	private final boolean filterResult;

	public RawVisionData(Pose3d estimatedPose, double timestamp, boolean isDataValid) {
		this.estimatedPose = estimatedPose;
		this.timestamp = timestamp;
		this.filterResult = isDataValid;
	}

	public Pose3d getEstimatedPose() {
		return estimatedPose;
	}

	public double getTimestamp() {
		return timestamp;
	}

	public boolean shallDataBeFiltered() {
		return filterResult;
	}

}
