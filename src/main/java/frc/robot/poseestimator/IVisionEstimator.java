package frc.robot.poseestimator;

import org.wpilib.math.geometry.Pose2d;
import frc.robot.vision.RobotPoseObservation;

public interface IVisionEstimator {

	void updateVision(RobotPoseObservation... robotPoseVisionData);

	Pose2d getEstimatedPose();

}
