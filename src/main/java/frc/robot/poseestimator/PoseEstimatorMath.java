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
        boolean isGyroConnected = observation.getGyroAngle() != null;
        if (isGyroConnected) {
            twist = updateDeltaTheta(twist, observation, lastGyroAngle);
        }
        return twist;
    }

    public static Twist2d updateDeltaTheta(Twist2d twist, OdometryObservation observation, Rotation2d lastGyroAngle) {
        return new Twist2d(
                twist.dx,
                twist.dy,
                observation.getGyroAngle().minus(lastGyroAngle).getRadians()
        );
    }

    public static double[] getSquaredVisionMatrix(VisionObservation observation) {
        int rows = observation.getStandardDeviations().getNumRows();
        double[] r = new double[rows];
        for (int row = 0; row < rows; ++row) {
            double standardDeviationInRow = observation.getStandardDeviations().get(row, 0);
            r[row] = Math.pow(standardDeviationInRow,PoseEstimatorConstants.KALMAN_EXPONENT);
        }
        return r;
    }

    public static Matrix<N3, N3> kalmanFilterAlgorithm(double[] squaredVisionMatrix, Matrix<N3, N1> standardDeviations) {
        Matrix<N3, N3> visionK = new Matrix<>(Nat.N3(), Nat.N3());
        for (int row = 0; row < visionK.getNumRows(); ++row) {
            double standardDeviation = standardDeviations.get(row, 0);
            if (standardDeviation == 0.0) {
                visionK.set(row, row, 0.0);
            }
            else {
                visionK.set(row, row, standardDeviation / (standardDeviation + Math.sqrt(standardDeviation * squaredVisionMatrix[row])));
            }
        }
        return visionK;
    }

    public static Transform2d useKalmanOnTransform(VisionObservation observation, Pose2d estimateAtTime, Matrix<N3, N1> standardDeviations) {
        double[] squaredVisionMatrix = PoseEstimatorMath.getSquaredVisionMatrix(observation);
        Matrix<N3, N3> visionK = PoseEstimatorMath.kalmanFilterAlgorithm(squaredVisionMatrix, standardDeviations);
        Transform2d diffFromOdometry = new Transform2d(estimateAtTime, observation.getVisionPose());
        return scaleDiffFromKalman(diffFromOdometry, visionK);
    }

    public static Transform2d scaleDiffFromKalman(Transform2d diffFromOdometry, Matrix<N3, N3> visionK) {
        Matrix<N3, N1> kTimesDiff = visionK.times(
                VecBuilder.fill(diffFromOdometry.getX(), diffFromOdometry.getY(), diffFromOdometry.getRotation().getRadians())
        );
        return new Transform2d(
                kTimesDiff.get(0, 0),
                kTimesDiff.get(1, 0),
                Rotation2d.fromRadians(kTimesDiff.get(2, 0))
        );
    }

    public static Pose2d combineVisionObservationAndOdometrySample(
            Optional<Pose2d> sample,
            VisionObservation observation,
            Pose2d estimatedPose,
            Pose2d odometryPose,
            Matrix<N3, N1> standardDeviations
    ) {
        Transform2d sampleToOdometryTransform = new Transform2d(sample.get(), odometryPose);
        Transform2d odometryToSampleTransform = new Transform2d(odometryPose, sample.get());
        Pose2d estimateAtTime = estimatedPose.plus(odometryToSampleTransform);
        return estimateAtTime
                .plus(PoseEstimatorMath.useKalmanOnTransform(observation, estimateAtTime, standardDeviations))
                .plus(sampleToOdometryTransform);
    }

}
