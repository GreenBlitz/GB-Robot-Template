package frc.robot.poseestimation.wpilibposeestimator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import frc.robot.RobotContainer;
import frc.robot.poseestimation.IPoseEstimator;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.utils.allianceutils.AlliancePose2d;
import org.littletonrobotics.junction.Logger;

public class WPILIBOdometry implements IPoseEstimator {

    private final SwerveDriveOdometry odometry;

    public WPILIBOdometry() {
        odometry = new SwerveDriveOdometry(SwerveConstants.KINEMATICS,
                RobotContainer.SWERVE.getHeading(),
                RobotContainer.SWERVE.getModulePositionsForWpilibPoseEstimator(),
                WPILIBOdometryConstants.DEFAULT_STARTING_POSE.toBlueAlliancePose()
        );
    }

    @Override
    public void update() {
        odometry.update(RobotContainer.SWERVE.getHeading(), RobotContainer.SWERVE.getModulePositionsForWpilibPoseEstimator());
        logPose();
    }

    @Override
    public void logPose() {
        Logger.recordOutput("Robot Odometry Position", getCurrentPose().toBlueAlliancePose());
    }

    @Override
    public void setHeading(Rotation2d heading) {
        resetPose(AlliancePose2d.fromBlueAlliancePose(odometry.getPoseMeters().getTranslation(), heading));
    }

    @Override
    public void resetPose(AlliancePose2d alliancePose2d) {
        odometry.resetPosition(RobotContainer.SWERVE.getHeading(),
                RobotContainer.SWERVE.getModulePositionsForWpilibPoseEstimator(),
                alliancePose2d.toBlueAlliancePose()
        );
    }

    @Override
    public AlliancePose2d getCurrentPose() {
        return AlliancePose2d.fromBlueAlliancePose(odometry.getPoseMeters());
    }

}
