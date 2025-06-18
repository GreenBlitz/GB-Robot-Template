package frc.robot.newvision.interfaces;

import frc.robot.newvision.RobotPoseObservation;

import java.util.Optional;

public interface IndependentRobotPoseSupplyingCamera {

	Optional<RobotPoseObservation> getRobotPose();

}
