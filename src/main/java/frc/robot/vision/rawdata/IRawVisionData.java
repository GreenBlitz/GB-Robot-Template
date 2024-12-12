package frc.robot.vision.rawdata;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.poseestimator.observations.IRobotPoseVisionObservation;

public interface IRawVisionData {

	public Pose3d getEstimatedPose();

	public double getTimestamp();

}
