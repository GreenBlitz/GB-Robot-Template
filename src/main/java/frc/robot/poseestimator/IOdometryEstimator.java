package frc.robot.poseestimator;

import frc.utils.costomUnits.Angle2d;
import org.wpilib.math.geometry.Pose2d;

public interface IOdometryEstimator {

	void updateOdometry(OdometryData[] odometryData);

	void updateOdometry(OdometryData odometryData);

	void resetPose(OdometryData odometryData, Pose2d poseMeters);

	Pose2d getOdometryPose();

	void setHeading(Angle2d newHeading);

}
