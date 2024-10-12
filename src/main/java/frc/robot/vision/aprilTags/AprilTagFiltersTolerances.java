package frc.robot.vision.aprilTags;

import edu.wpi.first.math.geometry.Rotation2d;

public record AprilTagFiltersTolerances(
	double aprilTagHeightToleranceMeters,
	double normalizedPositionTolerance,
	double normalizedRotationTolerance,
	Rotation2d rollTolerance,
	Rotation2d pitchTolerance,
	double robotToGroundToleranceMeters,
	double maximumAmbiguity,
	double maximumLatency
) {}
