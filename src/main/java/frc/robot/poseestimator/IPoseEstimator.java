package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.poseestimator.observations.OdometryObservation;
import frc.robot.poseestimator.observations.VisionObservation;

import java.util.List;
import java.util.Optional;

public interface IPoseEstimator extends IVisionEstimator, IOdometryEstimator {

	Pose2d getEstimatedPose();

	Optional<Pose2d> getEstimatedPoseAtTimeStamp(double timeStamp);

	void resetPose(Pose2d newPose);

	void updatePoseEstimator(List<OdometryObservation> odometryObservation, List<VisionObservation> visionObservations);

}
