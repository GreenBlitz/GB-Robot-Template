package frc.robot.poseestimator.photonvision.photoncamera;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.poseestimator.photonvision.PhotonTarget;

public record PhotonTargetData(
	Pose3d robotPose,
	PhotonTarget target,
	double timestamp,
	double ambiguity,
	double latency
) {}
