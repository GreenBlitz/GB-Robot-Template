package frc.robot.poseestimation;

import frc.robot.poseestimation.wpilibposeestimator.WPILIBOdometry;

public class PoseEstimatorFactory {
    
    public static IPoseEstimator createPoseEstimator() {
        return switch (PoseEstimatorConstants.POSE_ESTIMATOR_TYPE) {
            case WPILIB_ODOMETRY -> new WPILIBOdometry();
            case WPILIB_POSE_ESTIMATOR -> new WPILIBOdometry();
        };
    }

    public enum PoseEstimatorType {
        WPILIB_ODOMETRY,
        WPILIB_POSE_ESTIMATOR
    }
}
