package frc.robot.poseestimation.poseestimator;

import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.swerve.SwerveConstants;
import org.littletonrobotics.junction.Logger;

import static frc.robot.Robot.poseEstimator;
import static frc.robot.Robot.swerve;

/**
 * A class that estimates the robot's pose using team 6328's custom pose estimator.
 */
public class PoseEstimator implements AutoCloseable {

    private final Field2d field; //todo - maybe create field class, maybe delete field
    private final PoseEstimator6328 poseEstimator6328;

    public PoseEstimator(Pose2d startingPose) {
        this.field = new Field2d();
        this.poseEstimator6328 = PoseEstimator6328.getInstance();
        resetPose(startingPose);

        SmartDashboard.putData("Field", field);
        setLoggingPathToPaths();
    }

    private void setLoggingPathToPaths() {
        PathPlannerLogging.setLogActivePathCallback((pose) -> {//todo - move to pp util
            field.getObject("path").setPoses(pose);
            //todo - move to swerve
            Logger.recordOutput(SwerveConstants.SWERVE_LOG_PATH + "Current Path To Follow", pose.toArray(new Pose2d[0]));
        });
    }

    @Override
    public void close() {
        field.close();
    }

    public void periodic() {
        logCurrentPose();
        field.setRobotPose(getCurrentPose());
    }

    private void logCurrentPose() {
        Logger.recordOutput(PoseEstimatorConstants.POSE_LOG_PATH, getCurrentPose());
    }

    public void resetPose(Pose2d currentPose) {
        swerve.setHeading(currentPose.getRotation());
        poseEstimator6328.resetPose(currentPose);
    }

    public void resetHeading(Rotation2d targetAngle) {
        resetPose(new Pose2d(getCurrentPose().getTranslation(), targetAngle));
    }

    public Pose2d getCurrentPose() {
        return poseEstimator6328.getEstimatedPose();
    }


    public void updatePoseEstimatorOdometry() {
        int odometryUpdates = swerve.getOdometryTimeStepQueue().length;
        SwerveDriveWheelPositions[] swerveWheelPositions = new SwerveDriveWheelPositions[odometryUpdates];
        Rotation2d[] gyroRotations = new Rotation2d[odometryUpdates];

        for (int i = 0; i < odometryUpdates; i++) {
            swerveWheelPositions[i] = swerve.getSwerveWheelPositions(i);
            gyroRotations[i] = swerve.getOdometryYawUpdates()[i];
        }

        poseEstimator.updatePoseEstimatorStates(swerveWheelPositions, gyroRotations, swerve.getOdometryTimeStepQueue());
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
    private void updatePoseEstimatorStates(SwerveDriveWheelPositions[] swerveWheelPositions, Rotation2d[] gyroRotations, double[] timestamps) {
        for (int i = 0; i < swerveWheelPositions.length; i++) {
            poseEstimator6328.addOdometryObservation(new PoseEstimator6328.OdometryObservation(swerveWheelPositions[i], gyroRotations[i], timestamps[i]));
        }
    }

}
