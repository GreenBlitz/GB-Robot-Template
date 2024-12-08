package frc.robot.poseestimator.observations;

import edu.wpi.first.math.geometry.Pose3d;

public interface VisionObservation {

	Pose3d getEstimatedPose();

	double getTimestamp();

}
