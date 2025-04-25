package frc.robot.vision.sources;

import frc.robot.vision.RobotOrientationState;
import frc.robot.vision.data.AprilTagVisionData;

public interface RobotHeadingRequiringVisionSource extends VisionSource<AprilTagVisionData> {

	void updateRobotAngleValues(RobotOrientationState robotOrientationState);

}
