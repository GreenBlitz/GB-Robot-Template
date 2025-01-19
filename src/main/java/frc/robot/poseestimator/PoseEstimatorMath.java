package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.*;
import frc.robot.constants.MathConstants;

public class PoseEstimatorMath {

	public static double getKalmanRatio(double odometryStandardDeviation, double visionStandardDeviation) {
		return odometryStandardDeviation / (odometryStandardDeviation + visionStandardDeviation);
	}

	public static Rotation2d combineVisionHeadingToGyro(
		Rotation2d visionEstimatedHeading,
		Rotation2d gyroAngle,
		Rotation2d lastGyroAngle,
		Rotation2d currentEstimatedHeading,
		double gyroStandardDeviation,
		double visionStandardDeviation
	) {
		Rotation2d changeInAngleSinceVisionObservationWasTaken = PoseEstimatorMath.getAngleDistance(gyroAngle, lastGyroAngle);
		double visionAndGyroRatio = getKalmanRatio(gyroStandardDeviation, visionStandardDeviation);
		Rotation2d estimatedHeadingAtSampleTime = currentEstimatedHeading.minus(changeInAngleSinceVisionObservationWasTaken);
		Rotation2d differenceFromVisionAndEstimatedHeading = getAngleDistance(visionEstimatedHeading, estimatedHeadingAtSampleTime);
		Rotation2d scaledDifferenceToAddToEstimatedHeading = differenceFromVisionAndEstimatedHeading.times(visionAndGyroRatio);
		return currentEstimatedHeading.plus(scaledDifferenceToAddToEstimatedHeading);
	}

	public static Rotation2d getAngleDistance(Rotation2d angle1, Rotation2d angle2) {
		Rotation2d difference = angle1.minus(angle2);
		if (difference.getRadians() > Math.PI) {
			return Rotation2d.fromRadians(MathConstants.FULL_CIRCLE.getRadians() - difference.getRadians());
		}
		return difference;
	}

}
