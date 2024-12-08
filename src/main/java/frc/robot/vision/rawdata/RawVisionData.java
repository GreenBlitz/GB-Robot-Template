package frc.robot.vision.rawdata;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.poseestimator.observations.VisionObservation;

public class RawVisionData implements VisionObservation {

	protected final Pose3d estimatedPose;
	protected final double timestamp;

	public RawVisionData(Pose3d estimatedPose, double timestamp) {
		this.estimatedPose = estimatedPose;
		this.timestamp = timestamp;
	}

	public Pose3d getEstimatedPose() {
		return estimatedPose;
	}

	public double getTimestamp() {
		return timestamp;
	}

}
