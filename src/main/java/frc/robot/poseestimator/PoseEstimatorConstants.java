package frc.robot.poseestimator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class PoseEstimatorConstants {

	public static final double POSE_BUFFER_SIZE_SECONDS = 2.0;

	public static final int KALMAN_EXPONENT = 2;

	public static final Matrix<N3, N1> ODOMETRY_STANDARD_DEVIATIONS = VecBuilder.fill(0.003, 0.003, 0.0002);

}
