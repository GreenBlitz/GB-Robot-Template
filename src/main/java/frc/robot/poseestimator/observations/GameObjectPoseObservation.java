package frc.robot.poseestimator.observations;

import edu.wpi.first.math.geometry.Pose3d;

public record GameObjectPoseObservation(Pose3d pose, double timestamp) {}
