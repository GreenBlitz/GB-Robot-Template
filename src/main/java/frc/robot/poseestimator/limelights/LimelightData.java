package frc.robot.poseestimator.limelights;

import edu.wpi.first.math.geometry.Pose2d;

public record LimelightData(Pose2d EstimatedPosition, double AprilTagHeight, double DistanceFromAprilTag, double timeStamp) {}
