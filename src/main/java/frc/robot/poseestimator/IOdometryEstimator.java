package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public interface IOdometryEstimator {

	void updateOdometry(OdometryData[] odometryData);

	void updateOdometry(OdometryData odometryData);

	void resetPose(OdometryData odometryData, Pose2d poseMeters);

	Pose2d getOdometryPose();

	/**
	 * Returns the robot's yaw computed purely from swerve wheel kinematics,
	 * with no IMU input. Accumulates dtheta from toTwist2d() on every odometry
	 * update and is reset to the new pose's rotation on resetPose().
	 */
	Rotation2d getSwerveYaw();

	void setHeading(Rotation2d newHeading);

}
