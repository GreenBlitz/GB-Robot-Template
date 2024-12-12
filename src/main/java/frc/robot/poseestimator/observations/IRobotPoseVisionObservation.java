package frc.robot.poseestimator.observations;

import edu.wpi.first.math.geometry.Pose3d;

public interface IRobotPoseVisionObservation {

	Pose3d getEstimatedPose();

	double getTimestamp();

}
