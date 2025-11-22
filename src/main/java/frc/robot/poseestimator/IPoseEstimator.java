package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;


public interface IPoseEstimator extends IVisionEstimator, IOdometryEstimator {

	void resetPosition(double timestampSeconds, SwerveModulePosition[] wheelPositions, Pose2d poseMeters);

	Pose2d getEstimatedPose();

	Pose2d getEstimatedPoseAtTimestamp(double timestampSeconds);

	boolean isIMUOffsetCalibrated();

	void log();

}
