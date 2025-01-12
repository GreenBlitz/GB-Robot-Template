package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.*;
import frc.robot.poseestimator.helpers.StandardDeviations2D;
import frc.robot.vision.data.AprilTagVisionData;
import frc.robot.poseestimator.helpers.ProcessedVisionData;
import frc.robot.vision.data.VisionData;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;
import frc.robot.vision.data.VisionData;

public class PoseEstimationMath {

	public static StandardDeviations2D calculateStandardDeviationOfPose(AprilTagVisionData rawVisionData, Pose2d currentEstimatedPose) {
		double normalizedLimelightX = rawVisionData.getEstimatedPose().getX();
		double normalizedLimelightY = rawVisionData.getEstimatedPose().getY();
		double normalizedEstimatedX = currentEstimatedPose.getX();
		double normalizedEstimatedY = currentEstimatedPose.getY();
		return new StandardDeviations2D(
			calculateStandardDeviation(normalizedLimelightX, normalizedEstimatedX),
			calculateStandardDeviation(normalizedLimelightY, normalizedEstimatedY),
			calculateStandardDeviation(
				rawVisionData.getEstimatedPose().toPose2d().getRotation().getRadians(),
				currentEstimatedPose.getRotation().getRadians()
			)
		);
	}

	public static StandardDeviations2D calculateStandardDeviationOfPose(List<Pose2d> dataset) {
		double[] deconstructedStdDevs = applyFunctionOnPoseElements(dataset, PoseEstimationMath::calculateStandardDeviation);
		return new StandardDeviations2D(
			deconstructedStdDevs[Pose2dComponentsValue.Y_VALUE.getIndex()],
			deconstructedStdDevs[Pose2dComponentsValue.X_VALUE.getIndex()],
			deconstructedStdDevs[Pose2dComponentsValue.ROTATION_VALUE.getIndex()]
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

	public static StandardDeviations2D calculateStandardDeviationOfPose(VisionData rawVisionData, Pose2d refrencePose) {
		double visionX = rawVisionData.getEstimatedPose().getX();
		double visionY = rawVisionData.getEstimatedPose().getY();
		double visionAng = rawVisionData.getEstimatedPose().getRotation().toRotation2d().getRadians();
		double estimatedX = refrencePose.getX();
		double estimatedY = refrencePose.getY();
		double estimatedAng = refrencePose.getRotation().getRotations();
		return new StandardDeviations2D(
			calculateStandardDeviation(visionX, estimatedX),
			calculateStandardDeviation(visionY, estimatedY),
			calculateStandardDeviation(visionAng, estimatedAng)
		);
	}

	public static Pose2d meanOfPose(List<Pose2d> dataset) {
		double[] deconstructedPose = applyFunctionOnPoseElements(dataset, PoseEstimationMath::mean);
		return new Pose2d(
			deconstructedPose[Pose2dComponentsValue.X_VALUE.getIndex()],
			deconstructedPose[Pose2dComponentsValue.Y_VALUE.getIndex()],
			Rotation2d.fromRadians(deconstructedPose[Pose2dComponentsValue.ROTATION_VALUE.getIndex()])
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

	public static double deriveVisionData(VisionData starting, VisionData finish) {
		double dTime = finish.getTimestamp() - starting.getTimestamp();
		Pose2d startingPose = starting.getEstimatedPose().toPose2d();
		Pose2d finishingPose = finish.getEstimatedPose().toPose2d();
		return startingPose.minus(finishingPose).getTranslation().getNorm() / dTime;
	}

	public static double deriveTwist(Twist2d twist, double dt) {
		double d2D = Math.sqrt((Math.pow(twist.dx, 2) + Math.pow(twist.dy, 2)));
		return d2D / dt;
	}

	public static Pose2d weightedPoseMean(List<ProcessedVisionData> observations) {
		Pose2d poseMean = new Pose2d();
		double xWeightsSum = 0;
		double yWeightsSum = 0;
		double rotationDeviationSum = 0;

		for (ProcessedVisionData observation : observations) {
			double xWeight = 1 / observation.getStdDev().xStandardDeviations();
			double yWeight = 1 / observation.getStdDev().yStandardDeviations();
			double rotationWeight = 1 / observation.getStdDev().angleStandardDeviations();
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
