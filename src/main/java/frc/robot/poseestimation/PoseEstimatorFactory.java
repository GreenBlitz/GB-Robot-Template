package frc.robot.poseestimation;

import frc.robot.poseestimation.poseestimator6328.PoseEstimator6328;
import frc.robot.poseestimation.wpilibposeestimator.WPILIBOdometry;

public class PoseEstimatorFactory {

    public enum PoseEstimatorType {
        WPILIB_ODOMETRY,
        WPILIB_POSE_ESTIMATOR,
        POSE_ESTIMATOR_6328;
    }

    public static IPoseEstimator createPoseEstimator() {
        return switch (PoseEstimatorConstants.POSE_ESTIMATOR_TYPE) {
            case WPILIB_ODOMETRY -> new WPILIBOdometry();
            case WPILIB_POSE_ESTIMATOR -> new WPILIBOdometry();
            case POSE_ESTIMATOR_6328 -> new PoseEstimator6328();
        };
    }

}
