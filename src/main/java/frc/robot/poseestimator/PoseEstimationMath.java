package frc.robot.poseestimator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import frc.robot.constants.Field;
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

	public static double[] getKalmanRatio(double[] odometryStandardDeviations, double[] visionStandardDeviations) {
		double[] combinedStandardDeviations = new double[PoseArrayEntryValue.POSE_ARRAY_LENGTH];
		for (int i = 0; i < combinedStandardDeviations.length; i++) {
			double odometryStandardDeviation = odometryStandardDeviations[i];
			double visionStandardDeviation = visionStandardDeviations[i];
			combinedStandardDeviations[i] = getKalmanRatio(odometryStandardDeviation, visionStandardDeviation);
		}
		return combinedStandardDeviations;
	}

	public static double getKalmanRatio(double odometryStandardDeviation, double visionStandardDeviation) {
		return odometryStandardDeviation == 0
			? 0
			: (odometryStandardDeviation / (odometryStandardDeviation + visionStandardDeviation * Math.sqrt(odometryStandardDeviation)));
	}

	//@formatter:off
    public static Transform2d applyKalmanOnTransform(
            VisionObservation observation,
            Pose2d appliedVisionObservation,
            double[] odometryStandardDeviations
    ) {
        double[] combinedStandardDeviations = getKalmanRatio(observation.standardDeviations(), odometryStandardDeviations);
        Transform2d visionDifferenceFromOdometry = new Transform2d(appliedVisionObservation, observation.robotPose());
        return scaleDifferenceFromKalman(visionDifferenceFromOdometry, combinedStandardDeviations);
    }

	public static Transform2d scaleDifferenceFromKalman(
			Transform2d visionDifferenceFromOdometry,
			double[] combinedStandardDeviations
	) {
		return new Transform2d(
			visionDifferenceFromOdometry.getX() * combinedStandardDeviations[PoseArrayEntryValue.X_VALUE.getEntryValue()],
			visionDifferenceFromOdometry.getY() * combinedStandardDeviations[PoseArrayEntryValue.Y_VALUE.getEntryValue()],
			Rotation2d.fromRadians(
				visionDifferenceFromOdometry.getRotation().getRadians()
					* combinedStandardDeviations[PoseArrayEntryValue.ROTATION_VALUE.getEntryValue()]
			)
		);
	}
	//@formatter:on

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
			.plus(applyKalmanOnTransform(observation, appliedVisionObservation, odometryStandardDeviations));
		appliedVisionObservation = appliedVisionObservation.plus(poseDifferenceFromSample);
		return appliedVisionObservation;
	}

	public static double[] calculateStandardDeviationOfPose(LimelightRawData limelightRawData, Pose2d currentEstimatedPose) {
		double normalizedLimelightX = limelightRawData.estimatedPose().getX() / Field.LENGTH_METERS;
		double normalizedLimelightY = limelightRawData.estimatedPose().getY() / Field.WIDTH_METERS;
		double normalizedEstimatedX = currentEstimatedPose.getX() / Field.LENGTH_METERS;
		double normalizedEstimatedY = currentEstimatedPose.getY() / Field.WIDTH_METERS;
		return new double[] {
			calculateStandardDeviation(normalizedLimelightX, normalizedEstimatedX),
			calculateStandardDeviation(normalizedLimelightY, normalizedEstimatedY)};
	}

	private static double calculateStandardDeviation(double estimatedValue, double currentValue) {
		double mean = (estimatedValue + currentValue) / 2;
		return Math.sqrt((Math.pow(estimatedValue - mean, 2) + Math.pow(currentValue - mean, 2)) / 2);
	}

	public static Pose2d weightedPoseMean(List<VisionObservation> observations) {
		Pose2d poseMean = new Pose2d();
		double xWeightsSum = 0;
		double yWeightsSum = 0;
		double rotationDeviationSum = 0;

		for (VisionObservation observation : observations) {
			double xWeight = 1 / observation.standardDeviations()[PoseArrayEntryValue.X_VALUE.getEntryValue()];
			double yWeight = 1 / observation.standardDeviations()[PoseArrayEntryValue.Y_VALUE.getEntryValue()];
			double rotationWeight = 1 / observation.standardDeviations()[PoseArrayEntryValue.ROTATION_VALUE.getEntryValue()];
			xWeightsSum += xWeight;
			yWeightsSum += yWeight;
			rotationDeviationSum += rotationWeight;
			poseMean = new Pose2d(
				poseMean.getX() + observation.robotPose().getX() * xWeight,
				poseMean.getY() + observation.robotPose().getY() * yWeight,
				poseMean.getRotation().plus(observation.robotPose().getRotation()).times(rotationWeight)
			);
		}

		poseMean = new Pose2d(
			new Translation2d(poseMean.getX() / xWeightsSum, poseMean.getY() / yWeightsSum),
			poseMean.getRotation().div(rotationDeviationSum)
		);

		return poseMean;
	}

}
