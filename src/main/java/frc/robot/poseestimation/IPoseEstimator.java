package frc.robot.poseestimation;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.utils.allianceutils.AlliancePose2d;

public interface IPoseEstimator {

    void logPose();

    void update();

    void setHeading(Rotation2d heading);

    void resetPose(AlliancePose2d alliancePose2d);

    AlliancePose2d getCurrentPose();

}
