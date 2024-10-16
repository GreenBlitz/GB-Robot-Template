package frc.robot.vision.limelights;

import edu.wpi.first.math.geometry.Rotation2d;

public record LimelightFiltersTolerances(
	double normalizedPositionTolerance,
	double normalizedRotationTolerance,
	Rotation2d rollTolerance,
	Rotation2d pitchTolerance,
	double robotToGroundToleranceMeters
) {}
