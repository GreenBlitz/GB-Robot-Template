package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose3d;

public record RawVisionPoseData(Pose3d estimatedPose, double aprilTagHeight, double distanceFromAprilTag, double timestamp) {}
