package frc.robot.newvision.interfaces;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.utils.TimedValue;

public interface RobotYawRequiringCamera {

	void setRobotYaw(Rotation2d robotYaw);

	TimedValue<Pose3d> getYawRequiringRobotPose();

}
