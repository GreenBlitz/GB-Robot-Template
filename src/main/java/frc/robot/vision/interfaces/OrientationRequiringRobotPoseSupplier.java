package frc.robot.vision.interfaces;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.vision.RobotPoseObservation;

import java.util.Optional;

public interface OrientationRequiringRobotPoseSupplier {

	void setRobotOrientation(Rotation3d robotOrientation);

	default void setRobotOrientation(Rotation2d robotYaw) {
		setRobotOrientation(new Rotation3d(robotYaw));
	}

	Optional<RobotPoseObservation> getOrientationRequiringRobotPose();

}
