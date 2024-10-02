package frc.robot.poseestimator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import frc.robot.poseestimator.observations.VisionObservation;
import frc.robot.vision.limelights.LimelightRawData;

import java.util.List;

public class PoseEstimationMath {

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
		getKalmanRatioFromStandardDeviation(double[] odometryStandardDeviations, double[] visionStandardDeviations) {
		double[] combinedStandardDeviations = new double[3];
		for (int i = 0; i < combinedStandardDeviations.length; i++) {
			double odometryStandardDeviation = odometryStandardDeviations[i];
			double visionStandardDeviation = visionStandardDeviations[i];
			combinedStandardDeviations[i] = getKalmanRatioFromStandardDeviation(
				odometryStandardDeviation,
				visionStandardDeviation
			);
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
		double[] combinedStandardDeviations = PoseEstimationMath
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
			.plus(PoseEstimationMath.useKalmanOnTransform(observation, appliedVisionObservation, odometryStandardDeviations));
		appliedVisionObservation = appliedVisionObservation.plus(poseDifferenceFromSample);
		return appliedVisionObservation;
	}

	public static double[] calculateStandardDeviationOfPose(LimelightRawData limelightRawData, Pose2d currentEstimatedPose) {
		return new double[] {
			calculateStandardDeviationFromDifference(limelightRawData.estimatedPose().getX(), currentEstimatedPose.getX()),
			calculateStandardDeviationFromDifference(limelightRawData.estimatedPose().getY(), currentEstimatedPose.getY())};
	}

	private static double calculateStandardDeviationFromDifference(double estimatedValue, double currentValue) {
		double mean = (estimatedValue + currentValue) / 2;
		return Math.sqrt((Math.pow(estimatedValue - mean, 2) + Math.pow(currentValue - mean, 2)) / 2);
	}

	public static Pose2d weightedPoseMean(List<VisionObservation> observations) {
		Pose2d output = new Pose2d();
		double positionDeviationSum = 0;
		double rotationDeviationSum = 0;

		for (VisionObservation observation : observations) {
			double positionWeight = Math.pow(observation.standardDeviations()[PoseArrayEntryValue.X_VALUE.getEntryValue()], -1);
			double rotationWeight = Math.pow(observation.standardDeviations()[PoseArrayEntryValue.X_VALUE.getEntryValue()], -1);
			positionDeviationSum += positionWeight;
			rotationDeviationSum += rotationWeight;
			output = new Pose2d(
					output.getX() + observation.visionPose().getX() * positionWeight,
					output.getY() + observation.visionPose().getY() * positionWeight,
					output.getRotation().plus(observation.visionPose().getRotation()).times(rotationWeight)
			);
		}

		output = new Pose2d(output.getTranslation().div(positionDeviationSum), output.getRotation().div(rotationDeviationSum));

		return output;
	}

}
