package frc.robot.newvision.interfaces;

import frc.robot.newvision.RobotPoseObservation;

import java.util.Optional;

public interface IndependentRobotPoseSupplier {

	Optional<RobotPoseObservation> getIndependentRobotPose();

}
