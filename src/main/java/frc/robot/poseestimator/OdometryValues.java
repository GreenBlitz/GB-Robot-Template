package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;

public record OdometryValues(SwerveDriveKinematics kinematics, SwerveDriveWheelPositions wheelPositions, Rotation2d gyroAngle) {}
