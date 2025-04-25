package frc.robot.vision.sources;

import frc.robot.vision.OrientationState3D;
import frc.robot.vision.data.AprilTagVisionData;

public interface RobotHeadingRequiringVisionSource extends VisionSource<AprilTagVisionData> {

	void updateRobotAngleValues(OrientationState3D robotOrientationState);

}
