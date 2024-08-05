package frc.robot.poseestimator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;


public interface IVisionPose {

    void updateVision(VisionObservation visionObservation);

    Pose2d getVisionPose();

    void setStandardDeviations(Matrix<N3, N1> stdDevs);

}
