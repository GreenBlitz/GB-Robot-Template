package frc.robot.poseestimation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public record OdometryObservation(SwerveModulePosition[] wheelPositions, Rotation2d gyroAngle, double timestamp) {}
