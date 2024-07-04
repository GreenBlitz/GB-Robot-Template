package frc.robot.poseestimation.poseestimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;
import frc.robot.poseestimation.observations.OdometryObservation;


/**
 * A class that estimates the robot's pose using team 6328's custom pose estimator.
 */
public class PoseEstimator {

    public final PoseEstimator6328 poseEstimator6328;

    public PoseEstimator(Pose2d startingPose) {
        this.poseEstimator6328 = PoseEstimator6328.getInstance();
        resetPose(startingPose);
    }

    public void resetPose(Pose2d currentPose) {
        poseEstimator6328.resetPose(currentPose);
    }

    public void resetHeading(Rotation2d targetAngle) {
        resetPose(new Pose2d(Robot.getCurrentPose().getTranslation(), targetAngle));
    }


    /**
     * Updates the pose estimator with the given swerve wheel positions and gyro rotations.
     * This function accepts an array of swerve wheel positions and an array of gyro rotations because the odometry can be updated
     * at a faster rate than the main loop (which is 50 hertz).
     * This means you could have a couple of odometry updates per main loop, and you would want to update the pose estimator with
     * all of them.
     */
    public void updatePoseEstimatorOdometry(OdometryObservation[] odometryObservations) {
        for (OdometryObservation odometryObservation : odometryObservations) {
            poseEstimator6328.addOdometryObservation(odometryObservation);
        }
    }

}
