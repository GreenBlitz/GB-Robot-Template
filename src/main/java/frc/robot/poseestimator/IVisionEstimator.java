package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.newvision.RobotPoseObservation;

import java.util.List;
import java.util.Optional;

public interface IVisionEstimator {

	void updateVision(List<Optional<RobotPoseObservation>> robotPoseVisionData);

	Pose2d getEstimatedPose();

}
