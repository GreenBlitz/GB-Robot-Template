package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.constants.RobotHeadingEstimatorConstants;
import frc.utils.math.AngleMath;

public class PoseEstimatorMath {

	public static double getKalmanRatio(double odometryStandardDeviation, double visionStandardDeviation) {
		double ratio = odometryStandardDeviation / (odometryStandardDeviation + visionStandardDeviation);
		return Double.isInfinite(ratio) ? 1.0 / RobotHeadingEstimatorConstants.AMOUNT_OF_SOURCE_TYPES : ratio;
	}

	public static Rotation2d combineVisionHeadingAndGyro(
		Rotation2d visionEstimatedHeading,
		Rotation2d gyroAngle,
		Rotation2d lastGyroAngle,
		Rotation2d currentEstimatedHeading,
		double gyroStandardDeviation,
		double visionStandardDeviation
	) {
		Rotation2d changeInAngleSinceVisionDataWasObserved = AngleMath.getAngleDifference(gyroAngle, lastGyroAngle);
		double visionAndGyroRatio = getKalmanRatio(gyroStandardDeviation, visionStandardDeviation);
		Rotation2d estimatedHeadingAtSampleTime = currentEstimatedHeading.minus(changeInAngleSinceVisionDataWasObserved);
		Rotation2d differenceFromVisionAndEstimatedHeading = AngleMath.getAngleDifference(visionEstimatedHeading, estimatedHeadingAtSampleTime);
		Rotation2d scaledDifferenceToAddToEstimatedHeading = differenceFromVisionAndEstimatedHeading.times(visionAndGyroRatio);
		return currentEstimatedHeading.plus(scaledDifferenceToAddToEstimatedHeading);
	}

}
