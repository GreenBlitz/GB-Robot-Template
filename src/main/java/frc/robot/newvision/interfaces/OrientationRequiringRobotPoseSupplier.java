package frc.robot.newvision.interfaces;

import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.newvision.RobotPoseObservation;

import java.util.Optional;

public interface OrientationRequiringRobotPoseSupplier {

	void setRobotOrientation(Rotation3d robotOrientation);

	Optional<RobotPoseObservation> getOrientationRequiringRobotPose();

}
