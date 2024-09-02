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

	public static double[]
		combineStandardDeviationsMatrices(double[] visionStandardDeviations, double[] odometryStandardDeviations) {
		double[] combinedStandardDeviations = new double[3];
		for (int row = 0; row < combinedStandardDeviations.length; row++) {
			double odometryStandardDeviation = odometryStandardDeviations[row];
			double visionStandardDeviation = visionStandardDeviations[row];
			combinedStandardDeviations[row] = combineStandardDeviations(odometryStandardDeviation, visionStandardDeviation);
		}
		return combinedStandardDeviations;
	}

	public static double combineStandardDeviations(double odometryStandardDeviation, double visionStandardDeviation) {
		if (odometryStandardDeviation == 0) {
			return 0;
		}
		double squaredVisionStandardDeviation = visionStandardDeviation * visionStandardDeviation;
		return odometryStandardDeviation
			/ (odometryStandardDeviation + Math.sqrt(odometryStandardDeviation * squaredVisionStandardDeviation));
	}

	public static Transform2d
		useKalmanOnTransform(VisionObservation observation, Pose2d currentPoseEstimation, double[] odometryStandardDeviations) {
		double[] combinedStandardDeviations = PoseEstimatorMath
			.combineStandardDeviationsMatrices(observation.standardDeviations(), odometryStandardDeviations);
		Transform2d visionDifferenceFromOdometry = new Transform2d(currentPoseEstimation, observation.visionPose());
		return scaleDifferenceFromKalman(visionDifferenceFromOdometry, combinedStandardDeviations);
	}

	public static Transform2d
		scaleDifferenceFromKalman(Transform2d visionDifferenceFromOdometry, double[] combinedStandardDeviations) {
		double[] visionDifferenceFromOdometryMatrix = {
			visionDifferenceFromOdometry.getX(),
			visionDifferenceFromOdometry.getY(),
			visionDifferenceFromOdometry.getRotation().getRadians()};
		double[] standardDeviationsAppliedTransform = oneDimensionalHadamardProduct(
			combinedStandardDeviations,
			visionDifferenceFromOdometryMatrix
		);
		return new Transform2d(
			standardDeviationsAppliedTransform[0],
			standardDeviationsAppliedTransform[1],
			Rotation2d.fromRadians(standardDeviationsAppliedTransform[2])
		);
	}

	public static double[] oneDimensionalHadamardProduct(double[] array1, double[] array2) {
		double[] result = new double[Math.min(array1.length, array2.length)];
		for (int i = 0; i < result.length; i++) {
			result[i] = array1[i] * array2[i];
		}
		return result;
	}

	public static Pose2d combineVisionToOdometry(
		Pose2d odometryInterpolatedPoseSample,
		VisionObservation observation,
		Pose2d estimatedPose,
		Pose2d odometryPose,
		double[] odometryStandardDeviations
	) {
		Transform2d poseDifferenceFromSample = new Transform2d(odometryInterpolatedPoseSample, odometryPose);
		Transform2d invertedPoseDifferenceFromSample = poseDifferenceFromSample.inverse();
		Pose2d currentPoseEstimation = estimatedPose.plus(invertedPoseDifferenceFromSample);
		currentPoseEstimation = currentPoseEstimation
			.plus(PoseEstimatorMath.useKalmanOnTransform(observation, currentPoseEstimation, odometryStandardDeviations));
		currentPoseEstimation = currentPoseEstimation.plus(poseDifferenceFromSample);
		return currentPoseEstimation;
	}

}
