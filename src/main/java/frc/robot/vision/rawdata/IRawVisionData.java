package frc.robot.vision.rawdata;

import edu.wpi.first.math.geometry.Pose3d;

public interface IRawVisionData {

	public Pose3d getEstimatedPose();

	public double getTimestamp();

}
