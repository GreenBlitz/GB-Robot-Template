package frc.robot.poseestimation.poseestimator;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import frc.utils.allianceutils.AlliancePose2d;

public class PoseEstimatorConstants {

    public static double ODOMETRY_FREQUENCY_HERTZ = 200.0;

    protected static String POSE_LOG_PATH = "Robot Pose";


    /**
     * The vector represents how ambiguous each value is.
     * The first value represents how ambiguous is the x,
     * the second one for the y, and the third one is for the theta (rotation).
     * Increase these numbers to trust the estimate less.
     */
    protected static final Vector<N3> STATES_AMBIGUITY = VecBuilder.fill(0.003, 0.003, 0.0002);

    protected static final AlliancePose2d DEFAULT_POSE = AlliancePose2d.fromBlueAlliancePose(new Pose2d(5, 5, new Rotation2d()));

}
