package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public record OdometryValues(SwerveDriveKinematics kinematics, SwerveDriveKinematics wheelPositions, Rotation2d gyroAngle) {}
