package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.Optional;

public record OdometryData(SwerveModulePosition[] wheelPositions, Optional<Rotation2d> gyroAngle, double timestamp) {}
