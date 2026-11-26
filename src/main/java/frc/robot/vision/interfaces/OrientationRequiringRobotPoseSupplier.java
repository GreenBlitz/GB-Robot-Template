package frc.robot.vision.interfaces;

import frc.robot.vision.RobotPoseObservation;
import java.util.Optional;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;

public interface OrientationRequiringRobotPoseSupplier {

  void setRobotOrientation(Rotation3d robotOrientation);

  default void setRobotOrientation(Rotation2d robotYaw) {
    setRobotOrientation(new Rotation3d(robotYaw));
  }

  Optional<RobotPoseObservation> getOrientationRequiringRobotPose();
}
