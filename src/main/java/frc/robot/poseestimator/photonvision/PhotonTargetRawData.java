package frc.robot.poseestimator.photonvision;

import edu.wpi.first.math.geometry.Pose3d;

public record PhotonTargetRawData(Pose3d targetPose, double timestamp, double ambiguity, double latency) {}
