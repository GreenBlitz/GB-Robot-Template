package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.poseestimator.observations.OdometryObservation;
import frc.robot.poseestimator.observations.VisionObservation;

import java.util.List;
import java.util.Optional;

public class WpiLibPoseEstimator implements IPoseEstimator {

	private Pose2d estimatedPose;

	@Override
	public void resetPose(Pose2d newPose) {

	}

	@Override
	public Pose2d getEstimatedPose() {
		return null;
	}

	@Override
	public Pose2d getEstimatedPoseAtTimestamp(double timestamp) {
		return null;
	}

	@Override
	public void updateOdometry(OdometryObservation[] odometryObservations) {

	}

	@Override
	public void resetOdometry(SwerveModulePosition[] wheelPositions, Rotation2d gyroAngle, Pose2d robotPose) {

	}

	@Override
	public Pose2d getOdometryPose() {
		return null;
	}

	@Override
	public void setHeading(Rotation2d newHeading) {

	}

	@Override
	public void updateVision(List<VisionObservation> visionObservation) {

	}

	@Override
	public Optional<Pose2d> getVisionPose() {
		return Optional.empty();
	}
}
