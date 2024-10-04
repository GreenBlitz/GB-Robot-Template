package frc.robot.poseestimator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import frc.robot.constants.Field;
import frc.robot.poseestimator.observations.VisionObservation;
import frc.robot.vision.limelights.LimelightRawData;
import frc.utils.MathUtils;

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
	getKalmanRatio(double[] odometryStandardDeviations, double[] visionStandardDeviations) {
		double[] combinedStandardDeviations = new double[PoseArrayEntryValue.POSE_ARRAY_LENGTH];
		for (int i = 0; i < combinedStandardDeviations.length; i++) {
			double odometryStandardDeviation = odometryStandardDeviations[i];
			double visionStandardDeviation = visionStandardDeviations[i];
			combinedStandardDeviations[i] = getKalmanRatio(
				odometryStandardDeviation,
				visionStandardDeviation
			);
		}
		return combinedStandardDeviations;
	}

	public static double getKalmanRatio(double odometryStandardDeviation, double visionStandardDeviation) {
		if (odometryStandardDeviation == 0) {
			return 0;
		}
		double squaredVisionStandardDeviation = Math.pow(visionStandardDeviation, 2);
		return odometryStandardDeviation
			/ (odometryStandardDeviation + Math.sqrt(odometryStandardDeviation * squaredVisionStandardDeviation));
	}

	public static Transform2d applyKalmanOnTransform(
		VisionObservation observation,
		Pose2d appliedVisionObservation,
		double[] odometryStandardDeviations
	) {
		double[] combinedStandardDeviations = PoseEstimationMath
			.getKalmanRatio(observation.standardDeviations(), odometryStandardDeviations);
		Transform2d visionDifferenceFromOdometry = new Transform2d(appliedVisionObservation, observation.visionPose());
		return scaleDifferenceFromKalman(visionDifferenceFromOdometry, combinedStandardDeviations);
	}

	public static Transform2d
		scaleDifferenceFromKalman(Transform2d visionDifferenceFromOdometry, double[] combinedStandardDeviations) {
		double[] visionDifferenceFromOdometryMatrix = {
			visionDifferenceFromOdometry.getX(),
			visionDifferenceFromOdometry.getY(),
			visionDifferenceFromOdometry.getRotation().getRadians()};
		double[] standardDeviationsAppliedTransform = MathUtils.multiplyArrays(
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
			.plus(PoseEstimationMath.applyKalmanOnTransform(observation, appliedVisionObservation, odometryStandardDeviations));
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
