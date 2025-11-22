package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public interface IOdometryEstimator {

	void updateOdometry(OdometryData[] odometryData);

	void updateOdometry(OdometryData odometryData);

	void resetPose(double timestampSeconds, Rotation2d imuYaw, SwerveModulePosition[] wheelPositions, Pose2d robotPose);

	Pose2d getOdometryPose();

	void setHeading(Rotation2d newHeading);

}
