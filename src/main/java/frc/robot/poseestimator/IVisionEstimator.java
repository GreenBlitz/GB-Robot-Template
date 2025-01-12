package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.vision.data.AprilTagVisionData;

import java.util.List;

public interface IVisionEstimator {

	void updateVision(List<AprilTagVisionData> robotPoseVisionObservation);

	Pose2d getEstimatedPose();

}
