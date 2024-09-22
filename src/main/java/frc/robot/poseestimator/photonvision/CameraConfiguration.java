package frc.robot.poseestimator.photonvision;

import edu.wpi.first.math.geometry.Transform3d;

public record CameraConfiguration(String logPath, String name, PhotonTarget targetType, Transform3d cameraToRobot) {}
