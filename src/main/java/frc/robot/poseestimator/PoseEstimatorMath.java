package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.*;

public class PoseEstimatorMath {

	public static double getKalmanRatio(double odometryStandardDeviation, double visionStandardDeviation) {
		return odometryStandardDeviation / (odometryStandardDeviation + visionStandardDeviation);
	}

	public static Rotation2d combineVisionHeadingToGyro(
		Rotation2d visionEstimatedHeading,
		Rotation2d sampleDifferenceFromHeading,
		Rotation2d currentEstimatedHeading,
		double gyroStandardDeviation,
		double visionStandardDeviation
	) {
		double combinedStandardDeviation = getKalmanRatio(gyroStandardDeviation, visionStandardDeviation);
		Rotation2d estimatedHeadingAtSampleTime = currentEstimatedHeading.plus(sampleDifferenceFromHeading);
		Rotation2d differenceFromVisionAndEstimatedHeading = getShortestAngleDifference(visionEstimatedHeading, estimatedHeadingAtSampleTime);
		Rotation2d scaledDifferenceToAddToEstimatedHeading = differenceFromVisionAndEstimatedHeading.times(combinedStandardDeviation);
		return currentEstimatedHeading.plus(scaledDifferenceToAddToEstimatedHeading);
	}

	public static Rotation2d getShortestAngleDifference(Rotation2d angle1, Rotation2d angle2) {
		Rotation2d difference = angle1.minus(angle2);
		if (difference.getRadians() > Math.PI) {
			return Rotation2d.fromRadians(2 * Math.PI - difference.getRadians());
		}
		return difference;
	}

	public static double calculateStandardDeviation(double estimatedValue, double currentValue) {
		double mean = (estimatedValue + currentValue) / 2;
		return Math.sqrt((Math.pow(estimatedValue - mean, 2) + Math.pow(currentValue - mean, 2)) / 2);
	}

}
