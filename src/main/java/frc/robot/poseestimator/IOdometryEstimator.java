package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.Pose2d;
import frc.utils.angle2d.Angle2d;

public interface IOdometryEstimator {

	void updateOdometry(OdometryData[] odometryData);

	void updateOdometry(OdometryData odometryData);

	void resetPose(OdometryData odometryData, Pose2d poseMeters);

	Pose2d getOdometryPose();

	void setHeading(Angle2d newHeading);

}
