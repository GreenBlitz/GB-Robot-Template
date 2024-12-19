package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public record OdometryValues(SwerveDriveKinematics kinematics, SwerveModulePosition[] wheelPositions, Rotation2d gyroAngle) {}

