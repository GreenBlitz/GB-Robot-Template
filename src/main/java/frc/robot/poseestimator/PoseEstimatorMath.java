package frc.robot.poseestimator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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

	public static Matrix<N3, N3>
		combineStandardDeviationsMatrices(Matrix<N3, N1> visionStandardDeviations, Matrix<N3, N1> odometryStandardDeviations) {
		Matrix<N3, N3> combinedStandardDeviations = new Matrix<>(Nat.N3(), Nat.N3());
		for (int row = 0; row < combinedStandardDeviations.getNumRows(); row++) {
			double odometryStandardDeviation = odometryStandardDeviations.get(row, 0);
			double visionStandardDeviation = visionStandardDeviations.get(row, 0);
			combinedStandardDeviations
				.set(row, row, combineStandardDeviations(odometryStandardDeviation, visionStandardDeviation));
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

	public static Transform2d useKalmanOnTransform(
		VisionObservation observation,
		Pose2d currentPoseEstimation,
		Matrix<N3, N1> odometryStandardDeviations
	) {
		Matrix<N3, N3> combinedStandardDeviations = PoseEstimatorMath
			.combineStandardDeviationsMatrices(observation.standardDeviations(), odometryStandardDeviations);
		Transform2d visionDifferenceFromOdometry = new Transform2d(currentPoseEstimation, observation.visionPose());
		return scaleDifferenceFromKalman(visionDifferenceFromOdometry, combinedStandardDeviations);
	}

	public static Transform2d
		scaleDifferenceFromKalman(Transform2d visionDifferenceFromOdometry, Matrix<N3, N3> combinedStandardDeviations) {
		Matrix<
			N3,
			N1> visionDifferenceFromOdometryMatrix = VecBuilder.fill(
				visionDifferenceFromOdometry.getX(),
				visionDifferenceFromOdometry.getY(),
				visionDifferenceFromOdometry.getRotation().getRadians()
			);
		Matrix<N3, N1> visionMatrixTimesDifferences = combinedStandardDeviations.times(visionDifferenceFromOdometryMatrix);
		return new Transform2d(
			visionMatrixTimesDifferences.get(0, 0),
			visionMatrixTimesDifferences.get(1, 0),
			Rotation2d.fromRadians(visionMatrixTimesDifferences.get(2, 0))
		);
	}

	public static Pose2d combineVisionToOdometry(
		Pose2d odometryInterpolatedPoseSample,
		VisionObservation observation,
		Pose2d estimatedPose,
		Pose2d odometryPose,
		Matrix<N3, N1> odometryStandardDeviations
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
