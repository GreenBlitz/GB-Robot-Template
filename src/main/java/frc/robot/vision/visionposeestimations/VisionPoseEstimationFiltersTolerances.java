package frc.robot.vision.visionposeestimations;

import edu.wpi.first.math.geometry.Rotation2d;

public record VisionPoseEstimationFiltersTolerances(
	double aprilTagHeightToleranceMeters,
	double normalizedPositionTolerance,
	double normalizedRotationTolerance,
	Rotation2d rollTolerance,
	Rotation2d pitchTolerance,
	double robotToGroundToleranceMeters,
	double maximumAmbiguity,
	double maximumLatency
) {}
