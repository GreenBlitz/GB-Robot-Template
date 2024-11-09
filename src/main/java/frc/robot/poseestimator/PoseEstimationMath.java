package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.*;
import frc.robot.constants.Field;
import frc.robot.poseestimator.observations.VisionObservation;
import frc.robot.vision.RawVisionData;

import java.util.List;
import java.util.Optional;

public class PoseEstimationMath {

	public static Twist2d addGyroToTwist(Twist2d twist, Rotation2d currentGyroAngle, Rotation2d lastGyroAngle) {
		boolean hasGyroUpdated = currentGyroAngle != null;
		if (hasGyroUpdated && lastGyroAngle != null) {
			return updateChangeInAngle(twist, currentGyroAngle, lastGyroAngle);
		}
		return twist;
	}

	public static Twist2d updateChangeInAngle(Twist2d twist, Rotation2d currentGyroAngle, Rotation2d lastGyroAngle) {
		Rotation2d rotationDifference = currentGyroAngle.minus(lastGyroAngle);
		return new Twist2d(twist.dx, twist.dy, rotationDifference.getRadians());
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
		Transform2d sampleDifferenceFromPose = new Transform2d(odometryPose, odometryInterpolatedPoseSample);
		return estimatedPose.plus(applyKalmanOnTransform(observation, estimatedPose.plus(sampleDifferenceFromPose), odometryStandardDeviations));
	}

	public static double[] calculateStandardDeviationOfPose(RawVisionData rawVisionData, Pose2d currentEstimatedPose) {
		double normalizedLimelightX = rawVisionData.estimatedPose().getX() / Field.LENGTH_METERS;
		double normalizedLimelightY = rawVisionData.estimatedPose().getY() / Field.WIDTH_METERS;
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

	public static Pose2d poseMean(List<VisionObservation> observations) {
		Pose2d poseMean = new Pose2d();

		for (VisionObservation observation : observations) {
			poseMean = new Pose2d(
				poseMean.getX() + observation.robotPose().getX(),
				poseMean.getY() + observation.robotPose().getY(),
				poseMean.getRotation().plus(observation.robotPose().getRotation())
			);
		}

		int observationsCount = observations.size();
		return poseMean.div(observationsCount);
	}

	public static double fastSin(double x) {
		double sign = 1;

		x = x % (2 * Math.PI);
		if (x > Math.PI) {
			sign = -1;
			x -= Math.PI;
		} else if (x > Math.PI / 2) {
			x = Math.PI - x;
		}

		double taylorResult = x - Math.pow(x, 3) / 6 + Math.pow(x, 5) / 120;
		double integralFix = PoseEstimatorConstants.INTEGRAL_SIN_FIX;
		return taylorResult * sign + integralFix;
	}

	public static double fastCos(double x) {
		return fastSin(Math.PI / 2 - x);
	}

	public static Optional<Rotation2d> calculateAngleAverage(List<Rotation2d> estimatedHeadings) {
		double summedXComponent = 0;
		double summedYComponent = 0;
		for (Rotation2d heading : estimatedHeadings) {
			summedXComponent += fastCos(heading.getRadians());
			summedYComponent += fastSin(heading.getSin());
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

}
