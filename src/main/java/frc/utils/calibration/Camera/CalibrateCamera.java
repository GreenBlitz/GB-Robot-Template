package frc.utils.calibration.Camera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.utils.LimelightHelpers;
import frc.utils.math.AngleTransform;
import frc.utils.math.FieldMath;
import org.littletonrobotics.junction.Logger;

public class CalibrateCamera extends Command {

    private double cosX3DSum = 0, sumSinX3D = 0;
    private double sumCosY3D = 0, sumSinY3D = 0;
    private double sumCosZ3D = 0, sumSinZ3D = 0;
    private double sumCosPose = 0, sumSinPose = 0;

    private double sumTranslationX = 0;
    private double sumTranslationY = 0;
    private double sumTranslationZ = 0;

    private double sumPose2DX = 0;
    private double sumPose2Dy = 0;

    private final String logPathPrefix;
    private final String cameraName;
    private final int tagID;
    private final double robotXAxisDistanceFromTag;
    private final double middleOfTagHeight;
    private final Pose3d tagPoseFieldRelative;
    private final Pose3d cameraPoseFieldRelative;
    private Pose2d robotPoseFieldRelative;
    private Rotation3d finalCameraRotation;
    private Translation3d finalCameraTranslation;
    private final int neededNumberOfCycles;
    private int currentCycle = 0;

    public CalibrateCamera(
            AprilTagFields field,
            String logPathPrefix,
            String cameraName,
            int tagID,
            double robotXAxisDistanceFromTag,
            double middleOfTagHeight,
            int neededNumberOfCycles
    ) {
        this.cameraName = cameraName;
        this.middleOfTagHeight = middleOfTagHeight;
        this.logPathPrefix = logPathPrefix;
        this.tagID = tagID;
        this.robotXAxisDistanceFromTag = robotXAxisDistanceFromTag;
        this.neededNumberOfCycles = neededNumberOfCycles;
        LimelightHelpers.setCameraPose_RobotSpace(cameraName, 0, 0, 0, 0, 0, 0);
        this.tagPoseFieldRelative = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getTagPose(tagID).get();
        this.cameraPoseFieldRelative = LimelightHelpers.getBotPose3d_wpiBlue(cameraName);
    }

    private void logFunction() {
        Logger.recordOutput("CameraCalibration/" + logPathPrefix + "/tag/TagPoseFieldRelative", tagPoseFieldRelative);
        Logger.recordOutput("CameraCalibration/" + logPathPrefix + "/cam/cameraPoseFieldRelative", cameraPoseFieldRelative);
        Logger.recordOutput("CameraCalibration/" + logPathPrefix + "/robot/robotFieldRelative", robotPoseFieldRelative);
        Logger.recordOutput("CameraCalibration/" + logPathPrefix + "/solution/endTranslation", finalCameraTranslation);
    }

    public void initialize() {
        logFunction();

    }

    @Override
    public void execute() {
        logCameraPose(this.logPathPrefix, this.cameraName, this.tagID, this.robotXAxisDistanceFromTag, this.middleOfTagHeight);
        currentCycle += 1;
    }

    @Override
    public void end(boolean interrupted) {
        this.finalCameraTranslation = new Translation3d(
                sumTranslationX / this.neededNumberOfCycles,
                sumTranslationY / this.neededNumberOfCycles,
                this.sumTranslationZ / this.neededNumberOfCycles
        );
        this.robotPoseFieldRelative = new Pose2d(
                sumPose2DX / this.neededNumberOfCycles,
                sumPose2Dy / this.neededNumberOfCycles,
                Rotation2d.fromRadians(Math.atan2(sumSinPose / this.neededNumberOfCycles, sumCosPose / this.neededNumberOfCycles))
        );
        this.finalCameraRotation = new Rotation3d(
                Math.atan2(sumSinX3D / this.neededNumberOfCycles, cosX3DSum / this.neededNumberOfCycles),
                Math.atan2(sumSinY3D / this.neededNumberOfCycles, sumCosY3D / this.neededNumberOfCycles),
                Math.atan2(sumSinZ3D / this.neededNumberOfCycles, sumCosZ3D / this.neededNumberOfCycles)
        );
        super.end(interrupted);
    }

    public boolean isFinished() {
        return currentCycle == neededNumberOfCycles;
    }

    private void logCameraPose() {
        // add option to tags that are not perfectly aligned
        this.robotPoseFieldRelative = new Pose2d(
                // pose i combined from translation 2d the x and y of the filed and an angle / what angle ?
                tagPoseFieldRelative.getX() - robotXAxisDistanceFromTag,
                tagPoseFieldRelative.getY(),
                FieldMath.transformAngle(tagPoseFieldRelative.getRotation().toRotation2d(), AngleTransform.INVERT)
        );
        // limelight is funny so we invert pitch
        // represents make average
        this.finalCameraRotation = new Rotation3d(
                // represents three degrees
                cameraPoseFieldRelative.getRotation().getX(),
                -cameraPoseFieldRelative.getRotation().getY(),
                cameraPoseFieldRelative.getRotation().getZ() - robotPoseFieldRelative.getRotation().getRadians()
        );
        // limelight is funny so we invert y-axis
        this.finalCameraTranslation = new Translation3d(
                cameraPoseFieldRelative.getX() - robotPoseFieldRelative.getX(),
                -(cameraPoseFieldRelative.getY() - robotPoseFieldRelative.getY()),
                cameraPoseFieldRelative.getZ() - tagPoseFieldRelative.getZ() + middleOfTagHeight
        );
        sumObjectsValues();
    }

    private void sumObjectsValues() {
        sumTranslationX += cameraPoseFieldRelative.getX() - robotPoseFieldRelative.getX();
        sumTranslationY += -(cameraPoseFieldRelative.getY() - robotPoseFieldRelative.getY());
        sumTranslationZ += cameraPoseFieldRelative.getZ() - tagPoseFieldRelative.getZ() + middleOfTagHeight;

        sumPose2DX += tagPoseFieldRelative.getX() - robotXAxisDistanceFromTag;
        sumPose2Dy += tagPoseFieldRelative.getY();

        cosX3DSum += Math.cos(cameraPoseFieldRelative.getRotation().getX());
        sumSinX3D += Math.sin(cameraPoseFieldRelative.getRotation().getX());
        sumCosY3D += Math.cos(-cameraPoseFieldRelative.getRotation().getY());
        sumSinY3D += Math.sin(-cameraPoseFieldRelative.getRotation().getY());
        sumCosZ3D += Math.cos(cameraPoseFieldRelative.getRotation().getZ() - robotPoseFieldRelative.getRotation().getRadians());
        sumSinZ3D += Math.sin(cameraPoseFieldRelative.getRotation().getZ() - robotPoseFieldRelative.getRotation().getRadians());

        sumCosPose += Math.cos(FieldMath.transformAngle(tagPoseFieldRelative.getRotation().toRotation2d(), AngleTransform.INVERT).getRadians());
        sumSinPose += Math.sin(FieldMath.transformAngle(tagPoseFieldRelative.getRotation().toRotation2d(), AngleTransform.INVERT).getRadians());
    }

}
