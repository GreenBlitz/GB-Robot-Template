package frc.robot.poseestimator.observations;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.Optional;

public record OdometryObservation(SwerveModulePosition[] wheelPositions, Optional<Rotation2d> gyroAngle, double timestamp) {}
