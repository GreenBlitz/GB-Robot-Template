package frc.robot.vision.interfaces;

import frc.utils.angle2d.Angle2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.vision.RobotPoseObservation;

import java.util.Optional;

public interface OrientationRequiringRobotPoseSupplier {

	void setRobotOrientation(Rotation3d robotOrientation);

	default void setRobotOrientation(Angle2d robotYaw) {
		setRobotOrientation(new Rotation3d(robotYaw.toRotation2d()));
	}

	Optional<RobotPoseObservation> getOrientationRequiringRobotPose();

}
