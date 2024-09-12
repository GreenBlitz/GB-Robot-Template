package frc.robot.poseestimator.limelights;

import edu.wpi.first.math.geometry.Pose2d;

public record LimelightRawData(Pose2d estimatedPose, double aprilTagHeight, double distanceFromAprilTag, double timestamp) {}
