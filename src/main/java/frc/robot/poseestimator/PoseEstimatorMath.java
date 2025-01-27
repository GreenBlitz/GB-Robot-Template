package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.constants.MathConstants;
import frc.constants.RobotHeadingEstimatorConstants;

public class PoseEstimatorMath {

	public static double getKalmanRatio(double odometryStandardDeviation, double visionStandardDeviation) {
		double ratio = odometryStandardDeviation / (odometryStandardDeviation + visionStandardDeviation);
		return Double.isNaN(ratio) ? 1.0 / RobotHeadingEstimatorConstants.AMOUNT_OF_SOURCE_TYPES : ratio;
	}

	public static Rotation2d combineVisionHeadingAndGyro(
		Rotation2d visionEstimatedHeading,
		Rotation2d gyroAngle,
		Rotation2d lastGyroAngle,
		Rotation2d currentEstimatedHeading,
		double gyroStandardDeviation,
		double visionStandardDeviation
	) {
		Rotation2d changeInAngleSinceVisionDataWasObserved = PoseEstimatorMath.getAngleDifference(gyroAngle, lastGyroAngle);
		double visionAndGyroRatio = getKalmanRatio(gyroStandardDeviation, visionStandardDeviation);
		Rotation2d estimatedHeadingAtSampleTime = currentEstimatedHeading.minus(changeInAngleSinceVisionDataWasObserved);
		Rotation2d differenceFromVisionAndEstimatedHeading = getAngleDifference(visionEstimatedHeading, estimatedHeadingAtSampleTime);
		Rotation2d scaledDifferenceToAddToEstimatedHeading = differenceFromVisionAndEstimatedHeading.times(visionAndGyroRatio);
		return currentEstimatedHeading.plus(scaledDifferenceToAddToEstimatedHeading);
	}

	public static Rotation2d getAngleDifference(Rotation2d angle1, Rotation2d angle2) {
		Rotation2d difference = angle1.minus(angle2);
		if (difference.getRadians() > Math.PI) {
			return Rotation2d.fromRadians(MathConstants.FULL_CIRCLE.getRadians() - difference.getRadians());
		}
		return difference;
	}

}
