package frc.robot.poseestimation.wpilibposeestimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.utils.allianceutils.AlliancePose2d;
import org.littletonrobotics.junction.Logger;

public class Odometry {

    private final SwerveDriveOdometry odometry;

    public Odometry() {
        odometry = new SwerveDriveOdometry(
                SwerveConstants.KINEMATICS,
                RobotContainer.SWERVE.getHeading(),
                RobotContainer.SWERVE.getModulePositionsForWpilibPoseEstimator(),
                OdometryConstants.DEFAULT_STARTING_POSE.toBlueAlliancePose()
        );
    }

    public void update(Rotation2d getGyroAngle, SwerveModulePosition[] swerveModulePositions){
        odometry.update(getGyroAngle, swerveModulePositions);
        logPose();
    }

    public void logPose(){
        Logger.recordOutput("Robot Odometry Position", getCurrentPose().toBlueAlliancePose());
    }

    public void setHeading(Rotation2d heading) {
        resetPose(AlliancePose2d.fromBlueAlliancePose(odometry.getPoseMeters().getTranslation(), heading));
    }


    public void resetPose(AlliancePose2d alliancePose2d) {
        odometry.resetPosition(RobotContainer.SWERVE.getHeading(), RobotContainer.SWERVE.getModulePositionsForWpilibPoseEstimator(), alliancePose2d.toBlueAlliancePose());
    }


    public AlliancePose2d getCurrentPose() {
        return AlliancePose2d.fromBlueAlliancePose(odometry.getPoseMeters());
    }
}
