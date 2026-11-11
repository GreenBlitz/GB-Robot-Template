package frc.robot.vision.interfaces;

import frc.robot.vision.RobotPoseObservation;
import frc.utils.angle2d.Angle2d;
import java.util.Optional;
import org.wpilib.math.geometry.Rotation3d;

public interface OrientationRequiringRobotPoseSupplier {

  void setRobotOrientation(Rotation3d robotOrientation);

  default void setRobotOrientation(Angle2d robotYaw) {
    setRobotOrientation(new Rotation3d(robotYaw.toRotation2d()));
  }

  Optional<RobotPoseObservation> getOrientationRequiringRobotPose();
}
