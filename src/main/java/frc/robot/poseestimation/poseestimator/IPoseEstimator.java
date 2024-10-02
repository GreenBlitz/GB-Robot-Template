package frc.robot.poseestimation.poseestimator;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.poseestimation.observations.OdometryObservation;
import frc.robot.poseestimation.observations.VisionObservation;

import java.util.List;

public interface IPoseEstimator extends IVisionEstimator, IOdometryEstimator {

	Pose2d getEstimatedPose();

	void resetPose(Pose2d newPose);

	void updatePoseEstimator(List<OdometryObservation> odometryObservation, List<VisionObservation> visionObservations);

}
