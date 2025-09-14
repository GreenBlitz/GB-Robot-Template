package frc.robot.vision.interfaces;

import frc.robot.vision.RobotPoseObservation;

import java.util.Optional;

public interface IndependentRobotPoseSupplier {

	Optional<RobotPoseObservation> getIndependentRobotPose();

}
