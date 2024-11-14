package frc.robot.vision;

import edu.wpi.first.math.geometry.Rotation2d;

public record VisionFiltersTolerances(Rotation2d rollTolerance, Rotation2d pitchTolerance, double robotToGroundToleranceMeters) {}
