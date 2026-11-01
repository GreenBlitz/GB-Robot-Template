package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public interface IOdometryEstimator {

	void updateOdometry(OdometryData[] odometryData);

	void updateOdometry(OdometryData odometryData);

	void resetPose(OdometryData odometryData, Pose2d poseMeters);

	Pose2d getOdometryPose();

	void setHeading(Rotation2d newHeading);

}
