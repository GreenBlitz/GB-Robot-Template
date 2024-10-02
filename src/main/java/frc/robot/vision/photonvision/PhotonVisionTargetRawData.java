package frc.robot.vision.photonvision;

import edu.wpi.first.math.geometry.Pose3d;

public record PhotonVisionTargetRawData(
	String cameraName,
	Pose3d targetPose,
	double timestamp,
	double ambiguity,
	double latency
) {}
