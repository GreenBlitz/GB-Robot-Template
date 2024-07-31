package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.poseestimator.OdometryObservation;

public interface IOdometryPose {
    void updateOdometry(OdometryObservation odometryObservation);

    void resetOdometry(Transform2d gyroAngle,SwerveDriveWheelPositions wheelPositions,Pose2d pose);

    void getOdometryPosition();

    void setSTDev(Matrix<N3, N1> stdDevs);
}
