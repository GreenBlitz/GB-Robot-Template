package frc.robot.vision;

import edu.wpi.first.math.geometry.Rotation2d;

public record VisionFiltersTolerances(Rotation2d roll, Rotation2d pitch, double robotDistanceFromGroundMeters) {}
