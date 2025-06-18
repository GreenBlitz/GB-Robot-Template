package frc.robot.newvision.interfaces;

import edu.wpi.first.math.geometry.Pose3d;
import frc.utils.TimedValue;

public interface RobotPoseSupplyingCamera {

	TimedValue<Pose3d> getRobotPose();

}
