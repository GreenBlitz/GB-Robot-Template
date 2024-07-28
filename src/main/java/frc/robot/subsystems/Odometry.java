package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.poseestimator.OdometryObservation;

public interface Odometry {
    void update(OdometryObservation odometryObservation);

    void reset(Pose2d pose);

    void
}
