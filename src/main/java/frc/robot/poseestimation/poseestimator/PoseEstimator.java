package frc.robot.poseestimation.poseestimator;

import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.utils.allianceutils.AlliancePose2d;
import frc.utils.allianceutils.AllianceRotation2d;
import org.littletonrobotics.junction.Logger;

import static frc.robot.RobotContainer.POSE_ESTIMATOR;
import static frc.robot.RobotContainer.SWERVE;

/**
 * A class that estimates the robot's pose using team 6328's custom pose estimator.
 */
public class PoseEstimator implements AutoCloseable {

    private final Field2d field = new Field2d();
    private final PoseEstimator6328 swerveDrivePoseEstimator = PoseEstimator6328.getInstance();
    private AlliancePose2d robotPose;

    public PoseEstimator() {
        this.robotPose = PoseEstimatorConstants.DEFAULT_POSE;
        resetPose(robotPose);

        SmartDashboard.putData("Field", field);
        setLoggingPathToPaths();
    }

    private void setLoggingPathToPaths() {
        PathPlannerLogging.setLogActivePathCallback((pose) -> {
            field.getObject("path").setPoses(pose);
            Logger.recordOutput("Path", pose.toArray(new Pose2d[0]));
        });
    }

    @Override
    public void close() {
        field.close();
    }

    public void periodic() {
        robotPose = AlliancePose2d.fromBlueAlliancePose(swerveDrivePoseEstimator.getEstimatedPose());

        logCurrentPose();
        field.setRobotPose(getCurrentPose().getBlueAlliancePose());
    }

    private void logCurrentPose() {
        Logger.recordOutput(PoseEstimatorConstants.POSE_LOG_PATH, getCurrentPose().getBlueAlliancePose());
    }

    /**
     * Resets the pose estimator to the given pose, and the gyro to the given pose's heading.
     *
     * @param currentPose the pose to reset to, as an {@link AlliancePose2d}
     */
    public void resetPose(AlliancePose2d currentPose) {
        RobotContainer.SWERVE.setHeading(currentPose.getRotation());
        swerveDrivePoseEstimator.resetPose(currentPose.getBlueAlliancePose());
    }

    public void resetHeading(AllianceRotation2d targetAngle) {
        resetPose(AlliancePose2d.fromBlueAlliancePose(
                getCurrentPose().getBlueAlliancePose().getTranslation(),
                targetAngle.getBlueAllianceAngle()
        ));
    }

    /**
     * @return the estimated pose of the robot, as an {@link AlliancePose2d}
     */
    public AlliancePose2d getCurrentPose() {
        return robotPose;
    }


    public void updatePoseEstimatorOdometry() {
        int odometryUpdates = SWERVE.getOdometryTimeStepQueue().length;
        SwerveDriveWheelPositions[] swerveWheelPositions = new SwerveDriveWheelPositions[odometryUpdates];
        Rotation2d[] gyroRotations = new Rotation2d[odometryUpdates];

        for (int i = 0; i < odometryUpdates; i++) {
            swerveWheelPositions[i] = SWERVE.getSwerveWheelPositions(i);
            gyroRotations[i] = SWERVE.getOdometryYawUpdates()[i];
        }

        POSE_ESTIMATOR.updatePoseEstimatorStates(swerveWheelPositions, gyroRotations, SWERVE.getOdometryTimeStepQueue());
    }

    /**
     * Updates the pose estimator with the given swerve wheel positions and gyro rotations.
     * This function accepts an array of swerve wheel positions and an array of gyro rotations because the odometry can be updated
     * at a faster rate than the main loop (which is 50 hertz).
     * This means you could have a couple of odometry updates per main loop, and you would want to update the pose estimator with
     * all of them.
     *
     * @param swerveWheelPositions the swerve wheel positions accumulated since the last update
     * @param gyroRotations the gyro rotations accumulated since the last update
     */
    private void updatePoseEstimatorStates(SwerveDriveWheelPositions[] swerveWheelPositions, Rotation2d[] gyroRotations,
            double[] timestamps) {
        for (int i = 0; i < swerveWheelPositions.length; i++) {
            swerveDrivePoseEstimator.addOdometryObservation(new PoseEstimator6328.OdometryObservation(
                    swerveWheelPositions[i],
                    gyroRotations[i],
                    timestamps[i]
            ));
        }
    }

}
