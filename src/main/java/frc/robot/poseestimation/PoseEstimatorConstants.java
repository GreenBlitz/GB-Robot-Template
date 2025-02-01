package frc.robot.poseestimation;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;

public class PoseEstimatorConstants {

	protected static final String LOG_PATH = "PoseEstimator/";

	public static final double ODOMETRY_FREQUENCY_HERTZ = 250;

	/**
	 * The vector represents how ambiguous each value is. The first value represents how ambiguous is the x, the second one for the y, and the
	 * third one is for the theta (rotation). Increase these numbers to trust the estimate less.
	 */
	protected static final Vector<N3> ODOMETRY_STANDARD_DEVIATIONS = VecBuilder.fill(0.003, 0.003, 0.0002);

	protected static final Pose2d DEFAULT_POSE = new Pose2d(2, 5, new Rotation2d());

}
