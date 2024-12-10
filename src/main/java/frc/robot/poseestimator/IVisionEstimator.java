package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.poseestimator.observations.VisionRobotPoseObservation;

import java.util.List;
import java.util.Optional;

public interface IVisionEstimator {

	void updateVision(List<VisionRobotPoseObservation> visionRobotPoseObservation);

	Optional<Pose2d> getVisionPose();

}
