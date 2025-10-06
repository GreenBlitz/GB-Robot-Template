package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public interface IOdometryEstimator {

	void updateAllOdometry(OdometryData[] odometryDataThread, OdometryData odometryData);

	void updateOdometryThread(OdometryData odometryDataThread);

	void resetOdometry(SwerveModulePosition[] wheelPositions, Rotation2d gyroAngle, Pose2d robotPose);

	Pose2d getOdometryPoseThread();

	void setHeading(Rotation2d newHeading);

}
