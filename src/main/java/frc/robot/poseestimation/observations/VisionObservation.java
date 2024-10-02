package frc.robot.poseestimation.observations;

import edu.wpi.first.math.geometry.Pose2d;

public record VisionObservation(Pose2d visionPose, double[] standardDeviations, double timestamp) {}
