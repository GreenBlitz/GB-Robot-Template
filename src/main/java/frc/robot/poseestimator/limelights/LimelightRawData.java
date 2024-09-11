package frc.robot.poseestimator.limelights;

import edu.wpi.first.math.geometry.Pose2d;

public record LimelightRawData(Pose2d EstimatedPosition, double AprilTagHeight, double DistanceFromAprilTag, double timeStamp) {}
