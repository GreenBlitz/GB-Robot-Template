package frc.robot.poseestimator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.poseestimator.observations.OdometryObservation;
import frc.robot.poseestimator.observations.VisionObservation;
import java.util.Optional;

public class PoseEstimatorMath {

	public static Twist2d addGyroToTwistCalculations(OdometryObservation observation, Twist2d twist, Rotation2d lastGyroAngle) {
		boolean hasGyroUpdated = observation.gyroAngle() != null;
		if (hasGyroUpdated) {
			twist = updateChangeInAngle(twist, observation, lastGyroAngle);
		}
		return twist;
	}

	public static Twist2d updateChangeInAngle(Twist2d twist, OdometryObservation observation, Rotation2d lastGyroAngle) {
		return new Twist2d(twist.dx, twist.dy, observation.gyroAngle().minus(lastGyroAngle).getRadians());
	}

	public static double[] getSquaredVisionStandardDeviations(VisionObservation observation) {
		int numRows = observation.standardDeviations().getNumRows();
		double[] rows = new double[numRows];
		for (int row = 0; row < numRows; row++) {
			double standardDeviationInRow = observation.standardDeviations().get(row, 0);
			rows[row] = Math.pow(standardDeviationInRow, PoseEstimatorConstants.KALMAN_EXPONENT);
		}
		return rows;
	}

	public static Matrix<N3, N3> kalmanFilterAlgorithm(double[] squaredVisionMatrix, Matrix<N3, N1> odometryStandardDeviations) {
		Matrix<N3, N3> visionCalculationMatrix = new Matrix<>(Nat.N3(), Nat.N3());
		for (int row = 0; row < visionCalculationMatrix.getNumRows(); row++) {
			double odometryStandardDeviation = odometryStandardDeviations.get(row, 0);
			visionCalculationMatrix.set(row, row, kalmanFilterFunction(odometryStandardDeviation, squaredVisionMatrix[row]));
		}
		return visionCalculationMatrix;
	}

	public static double kalmanFilterFunction(double odometryStandardDeviation, double squaredVisionMatrixValue) {
		if (odometryStandardDeviation == 0) {
			return 0;
		}
		return odometryStandardDeviation / (odometryStandardDeviation + Math.sqrt(odometryStandardDeviation * squaredVisionMatrixValue));
	}

	public static Transform2d
		useKalmanOnTransform(VisionObservation observation, Pose2d currentPoseEstimation, Matrix<N3, N1> odometryStandardDeviations) {
		double[] squaredVisionStandardDeviations = PoseEstimatorMath.getSquaredVisionStandardDeviations(observation);
		Matrix<
			N3,
			N3> visionCalculationMatrix = PoseEstimatorMath.kalmanFilterAlgorithm(squaredVisionStandardDeviations, odometryStandardDeviations);
		Transform2d differenceFromOdometry = new Transform2d(currentPoseEstimation, observation.visionPose());
		return scaleDifferenceFromKalman(differenceFromOdometry, visionCalculationMatrix);
	}

	public static Transform2d scaleDifferenceFromKalman(Transform2d differenceFromOdometry, Matrix<N3, N3> visionCalculationMatrix) {
		Matrix<
			N3,
			N1> visionMatrixTimesDifferences = visionCalculationMatrix.times(
				VecBuilder.fill(differenceFromOdometry.getX(), differenceFromOdometry.getY(), differenceFromOdometry.getRotation().getRadians())
			);
		return new Transform2d(
			visionMatrixTimesDifferences.get(0, 0),
			visionMatrixTimesDifferences.get(1, 0),
			Rotation2d.fromRadians(visionMatrixTimesDifferences.get(2, 0))
		);
	}

	public static Pose2d combineVisionToOdometry(
		Optional<Pose2d> sample,
		VisionObservation observation,
		Pose2d estimatedPose,
		Pose2d odometryPose,
		Matrix<N3, N1> odometryStandardDeviations
	) {
		Transform2d sampleToOdometryTransform = new Transform2d(sample.get(), odometryPose);
		Transform2d odometryToSampleTransform = new Transform2d(odometryPose, sample.get());
		Pose2d currentPoseEstimation = estimatedPose.plus(odometryToSampleTransform);
		currentPoseEstimation = currentPoseEstimation
			.plus(PoseEstimatorMath.useKalmanOnTransform(observation, currentPoseEstimation, odometryStandardDeviations));
		currentPoseEstimation = currentPoseEstimation.plus(sampleToOdometryTransform);
		return currentPoseEstimation;
	}

}
