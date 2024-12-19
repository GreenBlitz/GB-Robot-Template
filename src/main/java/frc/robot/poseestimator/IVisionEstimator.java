package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.vision.rawdata.AprilTagVisionData;

import java.util.List;
import java.util.Optional;

public interface IVisionEstimator {

	void updateVision(List<AprilTagVisionData> robotPoseVisionObservation);

	Optional<Pose2d> getVisionPose();

}
