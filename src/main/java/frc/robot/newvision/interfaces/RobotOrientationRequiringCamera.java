package frc.robot.newvision.interfaces;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.utils.TimedValue;

import java.util.Optional;

public interface RobotOrientationRequiringCamera {

	void setRobotOrientation(Rotation3d robotOrientation);

	Optional<TimedValue<Pose2d>> getOrientationRequiringRobotPose();

}
