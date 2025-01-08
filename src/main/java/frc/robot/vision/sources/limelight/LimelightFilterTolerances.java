package frc.robot.vision.sources.limelight;

import edu.wpi.first.math.geometry.Rotation2d;

public record LimelightFilterTolerances(Rotation2d rollTolerance, Rotation2d pitchTolerance, double robotToGroundToleranceMeters) {
}
