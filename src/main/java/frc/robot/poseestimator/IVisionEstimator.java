package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.vision.rawdata.RawAprilTagVisionData;

import java.util.List;
import java.util.Optional;

public interface IVisionEstimator {

	void updateVision(List<RawAprilTagVisionData> robotPoseVisionObservation);

	Optional<Pose2d> getVisionPose();

}

