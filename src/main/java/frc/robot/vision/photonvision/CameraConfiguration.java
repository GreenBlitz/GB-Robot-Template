package frc.robot.vision.photonvision;

import edu.wpi.first.math.geometry.Transform3d;

public record CameraConfiguration(String name, PhotonVisionTarget targetType, Transform3d cameraToRobot) {}
