package frc.robot.vision.limelights;

import edu.wpi.first.math.geometry.Pose3d;

public record LimelightRawData(Pose3d estimatedPose, double aprilTagHeight, double distanceFromAprilTag, double timestamp) {}
