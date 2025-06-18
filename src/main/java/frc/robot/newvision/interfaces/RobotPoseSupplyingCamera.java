package frc.robot.newvision.interfaces;

import edu.wpi.first.math.geometry.Pose2d;
import frc.utils.TimedValue;

import java.util.Optional;

public interface RobotPoseSupplyingCamera {

	Optional<TimedValue<Pose2d>> getRobotPose();

}
