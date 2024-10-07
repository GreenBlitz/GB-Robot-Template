package frc.robot.poseestimator.observations;

import edu.wpi.first.math.geometry.Pose2d;

public record VisionObservation(Pose2d robotPose, double[] standardDeviations, double timestamp) {}
