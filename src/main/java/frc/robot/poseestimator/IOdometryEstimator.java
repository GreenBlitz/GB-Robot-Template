package frc.robot.poseestimator;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;

public interface IOdometryEstimator {

	void updateOdometry(OdometryData[] odometryData);

	void updateOdometry(OdometryData odometryData);

	void resetPose(
		double timestampSeconds,
		Rotation2d imuYaw,
		double imuAccelerationMagnitudeG,
		SwerveModulePosition[] wheelPositions,
		Pose2d poseMeters
	);

	Pose2d getOdometryPose();

	void setHeading(Rotation2d newHeading);

}
