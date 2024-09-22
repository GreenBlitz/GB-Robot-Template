package frc.robot.poseestimator.photonvision;

import edu.wpi.first.math.geometry.Transform3d;

public record CameraConfiguration(PhotonTarget targetType, String name, Transform3d cameraToRobot) {}
