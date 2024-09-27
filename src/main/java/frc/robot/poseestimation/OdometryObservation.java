package frc.robot.poseestimation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;

public record OdometryObservation(SwerveDriveWheelPositions wheelPositions, Rotation2d gyroAngle, double timestamp) {}
