package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.newvision.RobotPoseObservation;

import java.util.List;

public interface IVisionEstimator {

	void updateVision(List<RobotPoseObservation> robotPoseVisionData);

	Pose2d getEstimatedPose();

}
