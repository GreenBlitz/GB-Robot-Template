package frc.robot.poseestimator.helpers;

import edu.wpi.first.math.geometry.Pose2d;

public class ProcessedVisionData {

	StandardDeviations2D stdDev;
	double timestamp;
	Pose2d estimatedPose;

	public ProcessedVisionData(Pose2d estimatedPose, double timestamp, StandardDeviations2D stdDev) {
		this.estimatedPose = estimatedPose;
		this.timestamp = timestamp;
		this.stdDev = stdDev;
	}

	public StandardDeviations2D getStdDev() {
		return stdDev;
	}

	public Pose2d getEstimatedPose() {
		return estimatedPose;
	}

	public double getTimestamp() {
		return timestamp;
	}

}
