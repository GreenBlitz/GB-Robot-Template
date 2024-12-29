package frc.robot.poseestimator;

import frc.robot.vision.data.AprilTagVisionData;

import java.util.List;

public interface IVisionEstimator {

	void updateVision(List<AprilTagVisionData> robotPoseVisionObservation);

}
