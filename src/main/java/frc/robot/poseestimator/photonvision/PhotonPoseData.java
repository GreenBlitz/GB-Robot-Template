package frc.robot.poseestimator.photonvision;

import edu.wpi.first.math.geometry.Pose3d;

public record PhotonPoseData(Pose3d robotPose, PhotonTarget target, double timestamp, double ambiguity, double latency) {}
