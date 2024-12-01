package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.poseestimator.observations.VisionObservation;

import java.util.List;
import java.util.Optional;

public interface IVisionEstimator {

	void updateVision(List<VisionObservation> visionObservation);

	Optional<Pose2d> getVisionPose();

}
