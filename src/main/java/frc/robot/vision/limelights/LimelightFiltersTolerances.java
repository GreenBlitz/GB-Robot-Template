package frc.robot.vision.limelights;

import edu.wpi.first.math.geometry.Rotation2d;

public record LimelightFiltersTolerances(
	double aprilTagHeightToleranceMeters,
	double positionTolerance,
	double rotationTolerance,
	Rotation2d rollTolerance,
	Rotation2d pitchTolerance,
	double robotToGroundToleranceMeters
) {}
