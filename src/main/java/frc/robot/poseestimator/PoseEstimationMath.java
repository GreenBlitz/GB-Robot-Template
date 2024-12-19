package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.*;
import frc.robot.poseestimator.helpers.ProcessedVisionData;
import frc.robot.poseestimator.helpers.StandardDeviations2d;
import frc.robot.vision.rawdata.AprilTagVisionData;
import frc.robot.vision.rawdata.VisionData;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;

public class PoseEstimationMath {

	public static Twist2d addGyroToTwist(Twist2d twist, Rotation2d currentGyroAngle, Rotation2d lastGyroAngle) {
		if (currentGyroAngle != null && lastGyroAngle != null) {
			Rotation2d rotationDifference = currentGyroAngle.minus(lastGyroAngle);
			return new Twist2d(twist.dx, twist.dy, rotationDifference.getRadians());
		}
		return twist;
	}

	public static StandardDeviations2d getKalmanRatio(StandardDeviations2d odometryStandardDeviations, StandardDeviations2d visionStandardDeviations) {
		double[] combinedStandardDeviations = new double[Pose2dArrayValue.POSE_ARRAY_LENGTH];
		for (int i = 0; i < combinedStandardDeviations.length; i++) {
			double odometryStandardDeviation = odometryStandardDeviations.getDeviationByPoseIndex(i);
			double visionStandardDeviation = visionStandardDeviations.getDeviationByPoseIndex(i);
			combinedStandardDeviations[i] = getKalmanRatio(odometryStandardDeviation, visionStandardDeviation);
		}
		return new StandardDeviations2d(combinedStandardDeviations);
	}

	public static double getKalmanRatio(double odometryStandardDeviation, double visionStandardDeviation) {
		return odometryStandardDeviation / (odometryStandardDeviation + visionStandardDeviation);
	}

	public static Transform2d applyKalmanOnTransform(
		ProcessedVisionData observation,
		Pose2d appliedVisionObservation,
		StandardDeviations2d odometryStandardDeviations,
		StandardDeviations2d visionStandardDeviations
	) {
		StandardDeviations2d combinedStandardDeviations = getKalmanRatio(odometryStandardDeviations, visionStandardDeviations);
		Transform2d visionDifferenceFromOdometry = new Transform2d(appliedVisionObservation, observation.getEstimatedPose());
		return scaleDifferenceFromKalman(visionDifferenceFromOdometry, combinedStandardDeviations);
	}

	public static Transform2d scaleDifferenceFromKalman(Transform2d visionDifferenceFromOdometry, StandardDeviations2d combinedStandardDeviations) {
		return new Transform2d(
			visionDifferenceFromOdometry.getX() * combinedStandardDeviations.getxStandardDeviationsMeters(),
			visionDifferenceFromOdometry.getY() * combinedStandardDeviations.getyStandardDeviationsMeters(),
			Rotation2d.fromRadians(
				visionDifferenceFromOdometry.getRotation().getRadians()
					* combinedStandardDeviations.getThetaStandardDeviations().getRadians()
			)
		);
	}

	public static Pose2d combineVisionToOdometry(
		ProcessedVisionData observation,
		Pose2d odometryInterpolatedPoseSample,
		Pose2d estimatedPose,
		Pose2d odometryPose,
		StandardDeviations2d odometryStandardDeviations,
		StandardDeviations2d visionStandardDeviations
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

	public static StandardDeviations2d calculateStandardDeviationOfPose(List<Pose2d> dataset) {
		double[] deconstructedStdDevs = applyFunctionOnPoseElements(dataset, PoseEstimationMath::calculateStandardDeviation);
		return new StandardDeviations2d(
			deconstructedStdDevs[Pose2dArrayValue.Y_VALUE.getEntryValue()],
			deconstructedStdDevs[Pose2dArrayValue.X_VALUE.getEntryValue()],
			Rotation2d.fromRadians(deconstructedStdDevs[Pose2dArrayValue.ROTATION_VALUE.getEntryValue()])
		);
	}

	private static double[] applyFunctionOnPoseElements(List<Pose2d> dataset, Function<List<Double>, Double> function) {
		List<Double> XSet = new ArrayList<>();
		List<Double> YSet = new ArrayList<>();
		List<Double> AngleSet = new ArrayList<>();
		for (Pose2d data : dataset) {
			XSet.add(data.getX());
			YSet.add(data.getY());
			AngleSet.add(data.getRotation().getRadians());
		}
		return new double[] {function.apply(XSet), function.apply(YSet), function.apply(AngleSet)};
	}

	public static StandardDeviations2d calculateStandardDeviationOfPose(VisionData rawVisionData, Pose2d refrencePose) {
		double visionX = rawVisionData.getEstimatedPose().getX();
		double visionY = rawVisionData.getEstimatedPose().getY();
		double visionAng = rawVisionData.getEstimatedPose().getRotation().toRotation2d().getRadians();
		double estimatedX = refrencePose.getX();
		double estimatedY = refrencePose.getY();
		double estimatedAng = refrencePose.getRotation().getRotations();
		return new StandardDeviations2d(calculateStandardDeviation(visionX, estimatedX), calculateStandardDeviation(visionY, estimatedY), Rotation2d.fromRadians(calculateStandardDeviation(visionAng, estimatedAng)));
	}

	public static Pose2d meanOfPose(List<Pose2d> dataset) {
		double[] deconstructedPose = applyFunctionOnPoseElements(dataset, PoseEstimationMath::mean);
		return new Pose2d(
			deconstructedPose[Pose2dArrayValue.X_VALUE.getEntryValue()],
			deconstructedPose[Pose2dArrayValue.Y_VALUE.getEntryValue()],
			Rotation2d.fromRadians(deconstructedPose[Pose2dArrayValue.ROTATION_VALUE.getEntryValue()])
		);
	}

	public static double mean(List<Double> dataset) {
		double sum = 0;
		for (double data : dataset) {
			sum += data;
		}
		return sum / dataset.size();
	}

	public static double calculateStandardDeviation(List<Double> dataset) {
		double mean = mean(dataset);
		double squaredDeviation = 0;
		for (double data : dataset) {
			squaredDeviation += Math.pow(data - mean, 2);
		}
		return Math.sqrt(squaredDeviation / dataset.size());
	}

	private static double calculateStandardDeviation(double estimatedValue, double currentValue) {
		double mean = (estimatedValue + currentValue) / 2;
		return Math.sqrt((Math.pow(estimatedValue - mean, 2) + Math.pow(currentValue - mean, 2)) / 2);
	}

	public static Pose2d weightedPoseMean(List<ProcessedVisionData> observations) {
		Pose2d poseMean = new Pose2d();
		double xWeightsSum = 0;
		double yWeightsSum = 0;
		double rotationDeviationSum = 0;

		for (ProcessedVisionData observation : observations) {
			double xWeight = 1 / observation.getStdDev().getxStandardDeviationsMeters();
			double yWeight = 1 / observation.getStdDev().getyStandardDeviationsMeters();
			double rotationWeight = 1 / observation.getStdDev().getThetaStandardDeviations().getRadians();
			xWeightsSum += xWeight;
			yWeightsSum += yWeight;
			rotationDeviationSum += rotationWeight;
			poseMean = new Pose2d(
				poseMean.getX() + observation.getEstimatedPose().getX() * xWeight,
				poseMean.getY() + observation.getEstimatedPose().getY() * yWeight,
				poseMean.getRotation().plus(observation.getEstimatedPose().getRotation()).times(rotationWeight)
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

	public static double sigmoid(double x) {
		return Math.pow(1 + Math.pow(Math.E, -x), -1);
	}

	public static double hacovercosin(double x) {
		return (Math.sin(x) + 1) / 2.0;
	}

	public static ProcessedVisionData processVisionData(VisionData visionData, Pose2d referencePose) {
		return new ProcessedVisionData(
			visionData.getEstimatedPose().toPose2d(),
			visionData.getTimestamp(),
			PoseEstimationMath.calculateStandardDeviationOfPose(visionData, referencePose)
		);
	}

}
