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
    private int sumX = 0;
    private int sumY = 0;
    private int sumZ = 0;
    private String PathPrefix;
    private String cameraName;
    private int tagID;
    private double xRobotDistanceFromTag;
    private double middleOfTagHeight;
    private AprilTagFields field;
    private Pose3d tagPoseFieldRelative;
    private Pose3d cameraPoseFieldRelative;
    private Pose2d robotPoseFieldRelative;
    private Rotation3d endRot;
    private Translation3d endTranslation;
    private int numberOfCycles;
    private int counter = 0;

    public CalibrateCamera(AprilTagFields field, String PathPrefix, String cameraName, int tagID, double xRobotDistanceFromTag, double middleOfTagHeight, int numberOfCycles) {
        this.cameraName = cameraName;
        this.middleOfTagHeight = middleOfTagHeight;
        this.field = field;
        this.PathPrefix = PathPrefix;
        this.tagID = tagID;
        this.xRobotDistanceFromTag = xRobotDistanceFromTag;
        this.numberOfCycles = numberOfCycles;
        LimelightHelpers.setCameraPose_RobotSpace(cameraName, 0, 0, 0, 0, 0, 0);
        this.tagPoseFieldRelative = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getTagPose(tagID).get();
        this.cameraPoseFieldRelative = LimelightHelpers.getBotPose3d_wpiBlue(cameraName);
    }


    private void logFunction() {
        Logger.recordOutput("CameraCalibration/" + PathPrefix + "/tag/TagPoseFieldRelative", tagPoseFieldRelative);

        Logger.recordOutput("CameraCalibration/" + PathPrefix + "/cam/cameraPoseFieldRelative", cameraPoseFieldRelative);

        Logger.recordOutput("CameraCalibration/" + PathPrefix + "/robot/robotFieldRelative", robotPoseFieldRelative);

        Logger.recordOutput("CameraCalibration/" + PathPrefix + "/solution/endTranslation", endTranslation);
    }

    public void initialize() {

    }

    @Override
    public void execute() {
        logFunction();
        logCameraPose(this.PathPrefix, this.cameraName, this.tagID, this.xRobotDistanceFromTag, this.middleOfTagHeight);
        counter += 1;
    }

    @Override
    public void end(boolean interrupted) {
        this.endTranslation = new Translation3d(sumX / this.numberOfCycles, sumY / this.numberOfCycles, this.sumZ / this.numberOfCycles);
        super.end(interrupted);
    }

    public boolean isFinished() {
        return counter == numberOfCycles;
    }

    private void logCameraPose(String PathPrefix, String cameraName, int tagID, double xRobotDistanceFromTag, double middleOfTagHeight) {
        // add option to tags that are not perfectly aligned
        this.robotPoseFieldRelative = new Pose2d(
                tagPoseFieldRelative.getX() - xRobotDistanceFromTag,
                tagPoseFieldRelative.getY(),
                FieldMath.transformAngle(tagPoseFieldRelative.getRotation().toRotation2d(), AngleTransform.INVERT)
        );

        // limelight is funny so we invert pitch
      // represents make average
        this.endRot = new Rotation3d(
                cameraPoseFieldRelative.getRotation().getX(),
                -cameraPoseFieldRelative.getRotation().getY(),
                cameraPoseFieldRelative.getRotation().getZ() - robotPoseFieldRelative.getRotation().getRadians()
        );
        // limelight is funny so we invert y-axis
        sumX += cameraPoseFieldRelative.getX() - robotPoseFieldRelative.getX();
        sumY += -(cameraPoseFieldRelative.getY() - robotPoseFieldRelative.getY());
        sumZ += cameraPoseFieldRelative.getZ() - tagPoseFieldRelative.getZ() + middleOfTagHeight;
        //  );
// return the thing that creats the object every time         divide the some to a different function

    }


}
