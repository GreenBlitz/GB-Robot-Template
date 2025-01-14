package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.*;
import frc.robot.poseestimator.helpers.StandardDeviations2D;
import frc.robot.vision.data.AprilTagVisionData;
import frc.robot.vision.data.VisionData;

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

	@Deprecated
	private static double calculateStandardDeviation(double estimatedValue, double currentValue) {
		double mean = (estimatedValue + currentValue) / 2;
		return Math.sqrt((Math.pow(estimatedValue - mean, 2) + Math.pow(currentValue - mean, 2)) / 2);
	}

	public static double deriveVisionData(VisionData starting, VisionData finish) {
		double deltaTime = finish.getTimestamp() - starting.getTimestamp();
		Pose2d startingPose = starting.getEstimatedPose().toPose2d();
		Pose2d finishingPose = finish.getEstimatedPose().toPose2d();
		return startingPose.minus(finishingPose).getTranslation().getNorm() / deltaTime;
	}

	public static double deriveTwist(Twist2d twist, double deltaTime) {
		double distance = Math.sqrt((Math.pow(twist.dx, 2) + Math.pow(twist.dy, 2)));
		return distance / deltaTime;
	}

}
