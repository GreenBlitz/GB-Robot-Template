package frc.utils.math;

import edu.wpi.first.math.geometry.Rotation2d;
<<<<<<< HEAD
import frc.robot.poseestimator.helpers.RobotHeadingEstimator.RobotHeadingEstimatorConstants;
=======
import frc.robot.poseestimator.helpers.robotheadingestimator.RobotHeadingEstimatorConstants;
>>>>>>> template/master

public class PoseEstimationMath {

	public static double getKalmanRatio(double odometryStandardDeviation, double visionStandardDeviation) {
		double ratio = odometryStandardDeviation / (odometryStandardDeviation + visionStandardDeviation);
		return Double.isInfinite(ratio) ? 1.0 / RobotHeadingEstimatorConstants.AMOUNT_OF_SOURCE_TYPES : ratio;
	}

	public static Rotation2d combineVisionHeadingAndGyro(
		Rotation2d visionEstimatedHeadingAtTimeStamp,
		Rotation2d gyroAngleAtTimeStamp,
		Rotation2d lastGyroAngle,
		Rotation2d currentEstimatedHeading,
		double gyroStandardDeviation,
		double visionStandardDeviation
	) {
<<<<<<< HEAD
		Rotation2d changeInAngleSinceVisionDataWasObserved = AngleMath.getAngleDifference(gyroAngleAtTimeStamp, lastGyroAngle);
		double visionAndGyroRatio = getKalmanRatio(gyroStandardDeviation, visionStandardDeviation);
		Rotation2d estimatedHeadingAtSampleTime = currentEstimatedHeading.minus(changeInAngleSinceVisionDataWasObserved);
		Rotation2d differenceFromVisionAndEstimatedHeading = AngleMath
			.getAngleDifference(visionEstimatedHeadingAtTimeStamp, estimatedHeadingAtSampleTime);
=======
		Rotation2d changeInAngleSinceVisionDataWasObserved = AngleMath.getAngleDifferenceWrapped(gyroAngleAtTimeStamp, lastGyroAngle);
		double visionAndGyroRatio = getKalmanRatio(gyroStandardDeviation, visionStandardDeviation);
		Rotation2d estimatedHeadingAtSampleTime = currentEstimatedHeading.minus(changeInAngleSinceVisionDataWasObserved);
		Rotation2d differenceFromVisionAndEstimatedHeading = AngleMath
			.getAngleDifferenceWrapped(visionEstimatedHeadingAtTimeStamp, estimatedHeadingAtSampleTime);
>>>>>>> template/master
		Rotation2d scaledDifferenceToAddToEstimatedHeading = differenceFromVisionAndEstimatedHeading.times(visionAndGyroRatio);
		return currentEstimatedHeading.plus(scaledDifferenceToAddToEstimatedHeading);
	}

}
