package frc.robot.poseestimator.limelights;

import edu.wpi.first.math.geometry.Rotation2d;

public record SmartLimelightConfig(
	String Logpath,
	String HardwareLogpath,
	Rotation2d rotationTolerance,
	double PositionNormTolerance
) {}
