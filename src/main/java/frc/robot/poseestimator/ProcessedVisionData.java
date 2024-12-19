package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.poseestimator.helpers.StandardDeviations2d;

public class ProcessedVisionData {

	StandardDeviations2d stdDev;
	double timestamp;
	Pose2d estimatedPose;

	public ProcessedVisionData(Pose2d estimatedPose, double timestamp, StandardDeviations2d stdDev) {
		this.estimatedPose = estimatedPose;
		this.timestamp = timestamp;
		this.stdDev = stdDev;
	}

	public StandardDeviations2d getStdDev() {
		return stdDev;
	}

	public Pose2d getEstimatedPose() {
		return estimatedPose;
	}

	public double getTimestamp() {
		return timestamp;
	}

}
