package frc.robot.poseestimator.limelights;

import edu.wpi.first.math.geometry.Rotation2d;

public record VisionObservationFilteredConfig(
	String logPath,
	String hardwareLogPath,
	Rotation2d rotationTolerance,
	double positionNormTolerance,
	String[] limelightsNames
) {}