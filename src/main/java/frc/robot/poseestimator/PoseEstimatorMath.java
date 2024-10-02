package frc.robot.poseestimator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import frc.robot.poseestimator.observations.VisionObservation;

public class PoseEstimatorMath {

	public static Twist2d addGyroToTwist(Twist2d twist, Rotation2d currentGyroAngle, Rotation2d lastGyroAngle) {
		boolean hasGyroUpdated = currentGyroAngle != null;
		if (hasGyroUpdated) {
			return updateChangeInAngle(twist, currentGyroAngle, lastGyroAngle);
		}
		return twist;
	}

	public static Twist2d updateChangeInAngle(Twist2d twist, Rotation2d currentGyroAngle, Rotation2d lastGyroAngle) {
		double rotationDifference = currentGyroAngle.getRadians() - lastGyroAngle.getRadians();
		double wrappedRotationDifference = MathUtil.angleModulus(rotationDifference);
		return new Twist2d(twist.dx, twist.dy, wrappedRotationDifference);
	}

	public static double[] getKalmanRatioFromStandardDeviation(double[] odometryStandardDeviations, double[] visionStandardDeviations) {
		double[] combinedStandardDeviations = new double[3];
		for (int i = 0; i < combinedStandardDeviations.length; i++) {
			double odometryStandardDeviation = odometryStandardDeviations[i];
			double visionStandardDeviation = visionStandardDeviations[i];
			combinedStandardDeviations[i] = getKalmanRatioFromStandardDeviation(odometryStandardDeviation, visionStandardDeviation);
		}
		return combinedStandardDeviations;
	}

	public static double getKalmanRatioFromStandardDeviation(double odometryStandardDeviation, double visionStandardDeviation) {
		if (odometryStandardDeviation == 0) {
			return 0;
		}
		double squaredVisionStandardDeviation = Math.pow(visionStandardDeviation, 2);
		return odometryStandardDeviation
			/ (odometryStandardDeviation + Math.sqrt(odometryStandardDeviation * squaredVisionStandardDeviation));
	}

	public static double[] multiplyArrays(double[] array1, double[] array2) {
		double[] result = new double[Math.min(array1.length, array2.length)];
		for (int i = 0; i < result.length; i++) {
			result[i] = array1[i] * array2[i];
		}
		return result;
	}

	public static Transform2d useKalmanOnTransform(
		VisionObservation observation,
		Pose2d appliedVisionObservation,
		double[] odometryStandardDeviations
	) {
		double[] combinedStandardDeviations = PoseEstimatorMath
			.getKalmanRatioFromStandardDeviation(observation.standardDeviations(), odometryStandardDeviations);
		Transform2d visionDifferenceFromOdometry = new Transform2d(appliedVisionObservation, observation.visionPose());
		return scaleDifferenceFromKalman(visionDifferenceFromOdometry, combinedStandardDeviations);
	}

	public static Transform2d
		scaleDifferenceFromKalman(Transform2d visionDifferenceFromOdometry, double[] combinedStandardDeviations) {
		double[] visionDifferenceFromOdometryMatrix = {
			visionDifferenceFromOdometry.getX(),
			visionDifferenceFromOdometry.getY(),
			visionDifferenceFromOdometry.getRotation().getRadians()};
		double[] standardDeviationsAppliedTransform = multiplyArrays(
			combinedStandardDeviations,
			visionDifferenceFromOdometryMatrix
		);
		return new Transform2d(
			standardDeviationsAppliedTransform[PoseArrayEntryValue.X_VALUE.getEntryValue()],
			standardDeviationsAppliedTransform[PoseArrayEntryValue.Y_VALUE.getEntryValue()],
			Rotation2d.fromRadians(standardDeviationsAppliedTransform[PoseArrayEntryValue.ROTATION_VALUE.getEntryValue()])
		);
	}

	public static Pose2d combineVisionToOdometry(
		VisionObservation observation,
		Pose2d odometryInterpolatedPoseSample,
		Pose2d estimatedPose,
		Pose2d odometryPose,
		double[] odometryStandardDeviations
	) {
		Transform2d poseDifferenceFromSample = new Transform2d(odometryInterpolatedPoseSample, odometryPose);
		Transform2d sampleDifferenceFromPose = poseDifferenceFromSample.inverse();
		Pose2d appliedVisionObservation = estimatedPose.plus(sampleDifferenceFromPose);
		appliedVisionObservation = appliedVisionObservation
			.plus(PoseEstimatorMath.useKalmanOnTransform(observation, appliedVisionObservation, odometryStandardDeviations));
		appliedVisionObservation = appliedVisionObservation.plus(poseDifferenceFromSample);
		return appliedVisionObservation;
	}

}
