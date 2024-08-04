package frc.robot.poseestimator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import java.awt.geom.Point2D;

public interface IVisionPose {
    void updateVision(VisionObservation visionObservation);

    Point2D getVisionPose();

    void setStandardDeviations(Matrix<N3, N1> stdDevs);
}
