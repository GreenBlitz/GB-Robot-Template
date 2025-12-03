package frc.utils.calibration.Camera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.utils.LimelightHelpers;
import frc.utils.math.AngleTransform;
import frc.utils.math.FieldMath;
import org.littletonrobotics.junction.Logger;

public class CalibrateCamera extends Command {
   public CalibrateCamera()


    private void logFunction(){
       Logger.recordOutput("CameraCalibration/" + PathPrefix + "/tag/TagPoseFieldRelative", tagPoseFieldRelative);

       Logger.recordOutput("CameraCalibration/" + PathPrefix + "/cam/cameraPoseFieldRelative", cameraPoseFieldRelative);

       Logger.recordOutput("CameraCalibration/" + PathPrefix + "/robot/robotFieldRelative", robotPoseFieldRelative);

       Logger.recordOutput("CameraCalibration/" + PathPrefix + "/solution/endTranslation", endTranslation);
   }
    public void initialize(){

    }

    public void execute(){

    }

    public void end(){

    }
    public boolean isFinished(){

    }
    public void logCameraPose(String PathPrefix, String cameraName, int tagID, double xRobotDistanceFromTag, double middleOfTagHeight) {
        // TODO - make a command, do an average, make field param, add option to tags that are not perfectly aligned

        LimelightHelpers.setCameraPose_RobotSpace(cameraName, 0, 0, 0, 0, 0, 0);

        Pose3d tagPoseFieldRelative = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getTagPose(tagID).get();
        Pose3d cameraPoseFieldRelative = LimelightHelpers.getBotPose3d_wpiBlue(cameraName);

        Pose2d robotPoseFieldRelative = new Pose2d(
                tagPoseFieldRelative.getX() - xRobotDistanceFromTag,
                tagPoseFieldRelative.getY(),
                FieldMath.transformAngle(tagPoseFieldRelative.getRotation().toRotation2d(), AngleTransform.INVERT)
        );

        // limelight is funny so we invert pitch
        Rotation3d endRot = new Rotation3d(
                cameraPoseFieldRelative.getRotation().getX(),
                -cameraPoseFieldRelative.getRotation().getY(),
                cameraPoseFieldRelative.getRotation().getZ() - robotPoseFieldRelative.getRotation().getRadians()
        );

        // limelight is funny so we invert y-axis
        Translation3d endTranslation = new Translation3d(
                cameraPoseFieldRelative.getX() - robotPoseFieldRelative.getX(),
                -(cameraPoseFieldRelative.getY() - robotPoseFieldRelative.getY()),
                cameraPoseFieldRelative.getZ() - tagPoseFieldRelative.getZ() + middleOfTagHeight
        );


    }




}
