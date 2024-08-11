package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.Pose2d;

public interface IPoseEstimator extends IVisionPose,IOdometryPose{
    Pose2d getEstimatedPose();

    void restPose(Pose2d newPose);
}
