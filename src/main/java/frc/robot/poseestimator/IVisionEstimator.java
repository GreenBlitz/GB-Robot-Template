package frc.robot.poseestimator;

import frc.robot.vision.RobotPoseObservation;
import org.wpilib.math.geometry.Pose2d;

public interface IVisionEstimator {

  void updateVision(RobotPoseObservation... robotPoseVisionData);

  Pose2d getEstimatedPose();
}
