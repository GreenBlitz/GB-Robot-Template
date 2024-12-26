package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.*;
import frc.robot.constants.Field;
import frc.robot.vision.data.AprilTagVisionData;

import java.util.List;
import java.util.Optional;

public class PoseEstimationMath {

	public static Twist2d addGyroToTwist(Twist2d twist, Rotation2d currentGyroAngle, Rotation2d lastGyroAngle) {
		if (currentGyroAngle != null && lastGyroAngle != null) {
			Rotation2d rotationDifference = currentGyroAngle.minus(lastGyroAngle);
			return new Twist2d(twist.dx, twist.dy, rotationDifference.getRadians());
		}
		return twist;
	}

	public static double[] getKalmanRatio(double[] odometryStandardDeviations, double[] visionStandardDeviations) {
		double[] combinedStandardDeviations = new double[Pose2dArrayValue.POSE_ARRAY_LENGTH];
		for (int i = 0; i < combinedStandardDeviations.length; i++) {
			double odometryStandardDeviation = odometryStandardDeviations[i];
			double visionStandardDeviation = visionStandardDeviations[i];
			combinedStandardDeviations[i] = getKalmanRatio(odometryStandardDeviation, visionStandardDeviation);
		}
		return combinedStandardDeviations;
	}

	public static double getKalmanRatio(double odometryStandardDeviation, double visionStandardDeviation) {
		return odometryStandardDeviation / (odometryStandardDeviation + visionStandardDeviation);
	}

	public static Transform2d applyKalmanOnTransform(
		AprilTagVisionData observation,
		Pose2d appliedVisionObservation,
		double[] odometryStandardDeviations,
		double[] visionStandardDeviations
	) {
		double[] combinedStandardDeviations = getKalmanRatio(odometryStandardDeviations, visionStandardDeviations);
		Transform2d visionDifferenceFromOdometry = new Transform2d(appliedVisionObservation, observation.getEstimatedPose().toPose2d());
		return scaleDifferenceFromKalman(visionDifferenceFromOdometry, combinedStandardDeviations);
	}

	public static Transform2d scaleDifferenceFromKalman(Transform2d visionDifferenceFromOdometry, double[] combinedStandardDeviations) {
		return new Transform2d(
			visionDifferenceFromOdometry.getX() * combinedStandardDeviations[Pose2dArrayValue.X_VALUE.getValue()],
			visionDifferenceFromOdometry.getY() * combinedStandardDeviations[Pose2dArrayValue.Y_VALUE.getValue()],
			Rotation2d.fromRadians(
				visionDifferenceFromOdometry.getRotation().getRadians()
					* combinedStandardDeviations[Pose2dArrayValue.ROTATION_VALUE.getValue()]
			)
		);
	}

	public static Pose2d combineVisionToOdometry(
		AprilTagVisionData observation,
		Pose2d odometryInterpolatedPoseSample,
		Pose2d estimatedPose,
		Pose2d odometryPose,
		double[] odometryStandardDeviations,
		double[] visionStandardDeviations
	) {
		Transform2d sampleDifferenceFromPose = new Transform2d(odometryPose, odometryInterpolatedPoseSample);
		return estimatedPose.plus(
			applyKalmanOnTransform(
				observation,
				estimatedPose.plus(sampleDifferenceFromPose),
				odometryStandardDeviations,
				visionStandardDeviations
			)
		);
	}

	public static double[] calculateStandardDeviationOfPose(AprilTagVisionData rawVisionData, Pose2d currentEstimatedPose) {
		double normalizedLimelightX = rawVisionData.getEstimatedPose().getX() / Field.LENGTH_METERS;
		double normalizedLimelightY = rawVisionData.getEstimatedPose().getY() / Field.WIDTH_METERS;
		double normalizedEstimatedX = currentEstimatedPose.getX() / Field.LENGTH_METERS;
		double normalizedEstimatedY = currentEstimatedPose.getY() / Field.WIDTH_METERS;
		return new double[] {
			calculateStandardDeviation(normalizedLimelightX, normalizedEstimatedX),
			calculateStandardDeviation(normalizedLimelightY, normalizedEstimatedY),
			calculateStandardDeviation(
				rawVisionData.getEstimatedPose().toPose2d().getRotation().getRadians(),
				currentEstimatedPose.getRotation().getRadians()
			)};
	}

	private static double calculateStandardDeviation(double estimatedValue, double currentValue) {
		double mean = (estimatedValue + currentValue) / 2;
		return Math.sqrt((Math.pow(estimatedValue - mean, 2) + Math.pow(currentValue - mean, 2)) / 2);
	}

	public static Pose2d weightedPoseMean(List<AprilTagVisionData> observations, double[] visionStandardDeviations) {
		Pose2d poseMean = new Pose2d();
		double xWeightsSum = 0;
		double yWeightsSum = 0;
		double rotationDeviationSum = 0;

		for (AprilTagVisionData observation : observations) {
			double xWeight = 1 / visionStandardDeviations[Pose2dArrayValue.X_VALUE.getValue()];
			double yWeight = 1 / visionStandardDeviations[Pose2dArrayValue.Y_VALUE.getValue()];
			double rotationWeight = 1 / visionStandardDeviations[Pose2dArrayValue.ROTATION_VALUE.getValue()];
			xWeightsSum += xWeight;
			yWeightsSum += yWeight;
			rotationDeviationSum += rotationWeight;
			poseMean = new Pose2d(
				poseMean.getX() + observation.getEstimatedPose().getX() * xWeight,
				poseMean.getY() + observation.getEstimatedPose().getY() * yWeight,
				poseMean.getRotation().plus(observation.getEstimatedPose().toPose2d().getRotation()).times(rotationWeight)
			);
		}

		poseMean = new Pose2d(
			new Translation2d(poseMean.getX() / xWeightsSum, poseMean.getY() / yWeightsSum),
			poseMean.getRotation().div(rotationDeviationSum)
		);

		return poseMean;
	}

	public static Pose2d poseMean(List<AprilTagVisionData> observations) {
		Pose2d poseMean = new Pose2d();

		for (AprilTagVisionData observation : observations) {
			poseMean = new Pose2d(
				poseMean.getX() + observation.getEstimatedPose().getX(),
				poseMean.getY() + observation.getEstimatedPose().getY(),
				poseMean.getRotation().plus(observation.getEstimatedPose().toPose2d().getRotation())
			);
		}

		int observationsCount = observations.size();
		return poseMean.div(observationsCount);
	}

	public static Optional<Rotation2d> calculateAngleAverage(List<Rotation2d> estimatedHeadings) {
		double summedXComponent = 0;
		double summedYComponent = 0;
		for (Rotation2d heading : estimatedHeadings) {
			summedXComponent += heading.getCos();
			summedYComponent += heading.getSin();
		}
		if (summedXComponent == 0 || summedYComponent == 0 || estimatedHeadings.isEmpty()) {
			return Optional.empty();
		}
		summedXComponent /= estimatedHeadings.size();
		summedYComponent /= estimatedHeadings.size();
		return Optional.of(new Rotation2d(Math.atan2(summedYComponent, summedXComponent)));
	}

	public static double distanceBetweenPosesMeters(Pose2d first, Pose2d second) {
		return first.minus(second).getTranslation().getNorm();
	}

	public static double distanceBetweenPosesMeters(Pose3d first, Pose3d second) {
		return first.minus(second).getTranslation().getNorm();
	}

}
