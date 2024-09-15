package frc.robot.poseestimator.observations;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;

public record OdometryObservation(SwerveDriveWheelPositions wheelsPositions, Rotation2d gyroAngle, double timestamp) {}
