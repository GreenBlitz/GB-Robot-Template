package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.Pose2d;


public interface IPoseEstimator extends IVisionEstimator, IOdometryEstimator {

	void resetPose(Pose2d newPose);

	Pose2d getEstimatedPose();

	Pose2d getEstimatedPoseAtTimestamp(double timestamp);

}
