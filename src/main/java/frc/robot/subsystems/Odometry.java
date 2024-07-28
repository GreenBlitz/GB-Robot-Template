package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import frc.robot.poseestimator.OdometryObservation;

public interface Odometry {
    void updateOdometry(OdometryObservation odometryObservation);

    void resetOdometry(Transform2d gyroAngle,SwerveDriveWheelPositions wheelPositions,Pose2d pose);

    void getOdometryPosition();
}
