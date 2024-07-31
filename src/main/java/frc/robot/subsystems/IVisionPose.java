package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.poseestimator.VisionObservation;

public interface IVisionPose {
    void updateVision(VisionObservation visionObservation);

    void getVisionPose();

    void setSTDev(Matrix<N3, N1> stdDevs);
}
