package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose3d;

public record VisionRawData(String cameraName, Pose3d targetPose, double ambiguity, double timestamp) {}
