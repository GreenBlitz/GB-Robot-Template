package frc.robot.vision;

import frc.utils.math.StandardDeviations2D;
import org.wpilib.math.geometry.Pose2d;

public record RobotPoseObservation(
    double timestampSeconds, Pose2d robotPose, StandardDeviations2D stdDevs, String sourceCamera) {

  public RobotPoseObservation() {
    this(0, new Pose2d(), new StandardDeviations2D(), "none");
  }
}
