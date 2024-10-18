package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import frc.robot.poseestimator.observations.OdometryObservation;

import java.util.List;

public interface IOdometryEstimator {

	void updateOdometry(List<OdometryObservation> odometryObservation);

	void resetOdometry(SwerveDriveWheelPositions wheelPositions, Rotation2d gyroAngle, Pose2d robotPose);

	Pose2d getOdometryPose();

	void setOdometryStandardDeviations(double[] standardDeviations);

	void resetHeadingOffset(Rotation2d newHeading);

}
