package frc.robot.poseestimator;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.numbers.N3;
import frc.robot.poseestimator.observations.OdometryObservation;


public interface IOdometryEstimator {

    void updateOdometry(OdometryObservation odometryObservation);

    void resetOdometry(SwerveDriveWheelPositions wheelPositions, Rotation2d gyroAngle, Pose2d pose);

    Pose2d getOdometryPose();

    void setOdometryStandardDeviations(Vector<N3> standardDeviations);

}
