package frc.robot.poseestimator.helpers;

import edu.wpi.first.math.geometry.Pose2d;

public class ProcessedVisionData {

	double[] stdDev;
	double timestamp;
	Pose2d estimatedPose;

	public ProcessedVisionData(Pose2d estimatedPose, double timestamp, double[] stdDev) {
		this.estimatedPose = estimatedPose;
		this.timestamp = timestamp;
		this.stdDev = stdDev;
	}

	public double[] getStdDev() {
		return stdDev;
	}

	public Pose2d getEstimatedPose() {
		return estimatedPose;
	}

	public double getTimestamp() {
		return timestamp;
	}

}
