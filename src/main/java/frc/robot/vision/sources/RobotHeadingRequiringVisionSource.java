package frc.robot.vision.sources;

import frc.robot.vision.RobotAngleValues;
import frc.robot.vision.data.AprilTagVisionData;

public interface RobotHeadingRequiringVisionSource extends VisionSource<AprilTagVisionData> {

	void updateGyroAngleValues(RobotAngleValues robotAngleValues);

}
