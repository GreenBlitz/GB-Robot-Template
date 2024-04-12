package frc.robot.poseestimation.poseestimator6328;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import frc.robot.RobotContainer;
import frc.robot.poseestimation.IPoseEstimator;
import frc.utils.allianceutils.AlliancePose2d;

public class PoseEstimator6328 implements IPoseEstimator {

    @Override
    public void update() {
        final int odometryUpdates = RobotContainer.SWERVE.getOdometryUpdatesYawDegreesLength();
        final SwerveDriveWheelPositions[] swerveWheelPositions = new SwerveDriveWheelPositions[odometryUpdates];
        final Rotation2d[] gyroRotations = new Rotation2d[odometryUpdates];

        for (int i = 0; i < odometryUpdates; i++) {
            swerveWheelPositions[i] = RobotContainer.SWERVE.getSwerveWheelPositions(i);
            gyroRotations[i] = RobotContainer.SWERVE.getOdometryUpdatesYawValueAtIndex(i);
        }

        //       TODO: Add 6328 Pose Estimator
        //       RobotContainer.POSE_ESTIMATOR.updatePoseEstimatorStates(swerveWheelPositions, gyroRotations,
        // swerveInputs.odometryUpdatesTimestamp);
    }

    @Override
    public void logPose() {
    }

    @Override
    public void setHeading(Rotation2d heading) {
    }

    @Override
    public void resetPose(AlliancePose2d alliancePose2d) {
    }

    @Override
    public AlliancePose2d getCurrentPose() {
        return null;
    }
}
