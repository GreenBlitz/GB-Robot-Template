package frc.robot.poseestimator.observations;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public record VisionObservation(Pose2d visionPose, Matrix<N3, N1> standardDeviations, double timestamp) {}
