package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.orientation.OrientationSupplier;

import java.util.Optional;


public interface IPoseEstimator extends IVisionEstimator, IOdometryEstimator {

	void resetPose(Pose2d poseMeters);

	Pose2d getEstimatedPose();

	Optional<Pose2d> getEstimatedPoseAtTimestamp(double timestampSeconds);

	void setOrientationSupplier(OrientationSupplier orientationSupplier);

	boolean isOffsetCalibrated();

	void log();

}
