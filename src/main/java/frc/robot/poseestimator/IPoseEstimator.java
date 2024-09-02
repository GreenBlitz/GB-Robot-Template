package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.poseestimator.observations.OdometryObservation;
import frc.robot.poseestimator.observations.VisionObservation;

public interface IPoseEstimator extends IVisionEstimator, IOdometryEstimator {

    Pose2d getEstimatedPose();

    void resetPose(Pose2d newPose);

    void updatePoseEstimator();
}
