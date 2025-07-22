package frc.robot.vision.sources;

<<<<<<< HEAD
import frc.robot.vision.RobotAngleValues;
=======
import frc.robot.vision.OrientationState3D;
>>>>>>> template/master
import frc.robot.vision.data.AprilTagVisionData;

public interface RobotHeadingRequiringVisionSource extends VisionSource<AprilTagVisionData> {

<<<<<<< HEAD
	void updateRobotAngleValues(RobotAngleValues robotAngleValues);
=======
	void updateRobotAngleValues(OrientationState3D robotOrientationState);
>>>>>>> template/master

}
