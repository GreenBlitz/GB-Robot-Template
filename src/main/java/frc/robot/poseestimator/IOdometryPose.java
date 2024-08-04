package frc.robot.poseestimator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public interface IOdometryPose {
    void updateOdometry(OdometryObservation odometryObservation);

    void resetOdometry(Rotation2d gyroAngle,SwerveDriveWheelPositions wheelPositions,Pose2d pose);

    Pose2d getOdometryPosition();

    void setSTDev(Matrix<N3, N1> stdDevs);
}
