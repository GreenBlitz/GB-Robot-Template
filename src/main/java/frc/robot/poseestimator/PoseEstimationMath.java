package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.*;
import frc.robot.poseestimator.helpers.StandardDeviations2D;
import frc.robot.vision.data.AprilTagVisionData;

public class PoseEstimationMath {

	public static StandardDeviations2D calculateStandardDeviationOfPose(AprilTagVisionData rawVisionData, Pose2d currentEstimatedPose) {
		double normalizedLimelightX = rawVisionData.getEstimatedPose().getX();
		double normalizedLimelightY = rawVisionData.getEstimatedPose().getY();
		double normalizedEstimatedX = currentEstimatedPose.getX();
		double normalizedEstimatedY = currentEstimatedPose.getY();
		return new StandardDeviations2D(
			calculateStandardDeviation(normalizedLimelightX, normalizedEstimatedX),
			calculateStandardDeviation(normalizedLimelightY, normalizedEstimatedY),
			calculateStandardDeviation(
				rawVisionData.getEstimatedPose().toPose2d().getRotation().getRadians(),
				currentEstimatedPose.getRotation().getRadians()
			)
		);
	}

	private static double calculateStandardDeviation(double estimatedValue, double currentValue) {
		double mean = (estimatedValue + currentValue) / 2;
		return Math.sqrt((Math.pow(estimatedValue - mean, 2) + Math.pow(currentValue - mean, 2)) / 2);
	}

}
