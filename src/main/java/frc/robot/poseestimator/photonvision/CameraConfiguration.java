package frc.robot.poseestimator.photonvision;

import edu.wpi.first.math.geometry.Transform3d;

public record CameraConfiguration(String logPathPrefix, String name, PhotonVisionTarget targetType, Transform3d cameraToRobot) {}
