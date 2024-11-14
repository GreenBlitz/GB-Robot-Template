package frc.robot.poseestimator.observations;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public record OdometryObservation(SwerveModulePosition[] wheelsPositions, Rotation2d gyroAngle, double timestamp) {}
