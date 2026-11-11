package frc.robot.poseestimator;

import java.util.Optional;
import org.wpilib.math.geometry.Pose2d;

public interface IPoseEstimator extends IVisionEstimator, IOdometryEstimator {

  void resetPose(Pose2d poseMeters);

  Pose2d getEstimatedPose();

  Optional<Pose2d> getEstimatedPoseAtTimestamp(double timestampSeconds);

  boolean isIMUOffsetCalibrated();

  void log();
}
