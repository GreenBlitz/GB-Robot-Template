package frc.utils.calibration.Camera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.utils.limelight.LimelightHelpers;
import frc.utils.math.AngleTransform;
import frc.utils.math.FieldMath;
import org.littletonrobotics.junction.Logger;

public class CameraPositionCalibration  extends Command {

    private final String logPathPrefix;
    private final String cameraName;

    private final AprilTagFields field;
    private final int tagID;

    private final double robotXAxisDistanceFromTag;
    private final double middleOfTagHeight;
    private final Pose3d tagPoseFieldRelative;
    private final int neededNumberOfCycles;

    private Pose3d cameraPoseFieldRelative;
    private Pose2d robotPoseFieldRelative;

    private Rotation3d finalCameraRotation;
    private Translation3d finalCameraTranslation;
    private int currentCycle;
	private double cosX3DSum = 0, sinX3DSum = 0;
	private double cosY3DSum = 0, sinY3DSum = 0;
	private double cosZ3DSum = 0, sinZ3DSum = 0;

    private Translation3d translationSum;
    private Translation3d currentCameraTranslation;
	private Rotation3d currentCameraRotation;

	public CameraPositionCalibration (
		AprilTagFields field,
		String logPathPrefix,
		String cameraName,
		int tagID,
		double robotXAxisDistanceFromTag,
		double middleOfTagHeight,
		int neededNumberOfCycles
	) {
		this.field = field;
		this.cameraName = cameraName;
		this.middleOfTagHeight = middleOfTagHeight;
		this.logPathPrefix = logPathPrefix;
		this.tagID = tagID;
		this.robotXAxisDistanceFromTag = robotXAxisDistanceFromTag;
		this.neededNumberOfCycles = neededNumberOfCycles;
        this.tagPoseFieldRelative = AprilTagFieldLayout.loadField(field).getTagPose(tagID).get();
        this.robotPoseFieldRelative = new Pose2d(
                // tag must be either 180 or 0 deg to the filed
                tagPoseFieldRelative.getX() - robotXAxisDistanceFromTag,
                tagPoseFieldRelative.getY(),
                FieldMath.transformAngle(tagPoseFieldRelative.getRotation().toRotation2d(), AngleTransform.INVERT)
        );

        LimelightHelpers.setCameraPose_RobotSpace(cameraName, 0, 0, 0, 0, 0, 0);
        this.cameraPoseFieldRelative = LimelightHelpers.getBotPose3d_wpiBlue(cameraName);
        this.translationSum = new Translation3d();
        this.currentCameraTranslation = new Translation3d();
        this.currentCameraRotation = new Rotation3d();
	}

	private void logFunction() {
        Logger.recordOutput("CameraCalibration" + logPathPrefix + "/solution/endRotation", finalCameraRotation);
        Logger.recordOutput("CameraCalibration" + logPathPrefix + "/solution/endTranslation", finalCameraTranslation);
        Logger.recordOutput("CameraCalibration" + logPathPrefix + "/cameraPoseFieldRelative", cameraPoseFieldRelative);

    }

	public void initialize() {
        Logger.recordOutput("CameraCalibration/" + logPathPrefix + "/tag/TagPoseFieldRelative", tagPoseFieldRelative);
        Logger.recordOutput("CameraCalibration" + logPathPrefix + "/robot/robotFieldRelative", robotPoseFieldRelative);	}

	@Override
	public void execute() {
		cameraPoseFieldRelative = LimelightHelpers.getBotPose3d_wpiBlue(cameraName);
		calculateRobotRelativeCameraPosition();
        sumObjectsValues();
        currentCycle += 1;
	}

	@Override
	public void end(boolean interrupted) {
		finalCameraTranslation = translationSum.div(neededNumberOfCycles);
		finalCameraRotation = new Rotation3d(
			Math.atan2(sinX3DSum / neededNumberOfCycles, cosX3DSum / neededNumberOfCycles),
			Math.atan2(sinY3DSum / neededNumberOfCycles, cosY3DSum / neededNumberOfCycles),
			Math.atan2(sinZ3DSum / neededNumberOfCycles, cosZ3DSum / neededNumberOfCycles)
		);
        Logger.recordOutput("CameraCalibration" + logPathPrefix + "/solution/endTranslation", finalCameraTranslation);
        Logger.recordOutput("CameraCalibration" + logPathPrefix + "/solution/endRotation", finalCameraRotation);
		super.end(interrupted);
	}

	public boolean isFinished() {
		return currentCycle >= neededNumberOfCycles;
	}

	private void calculateRobotRelativeCameraPosition() {
		// add option to tags that are not perfectly aligned
		currentCameraRotation = new Rotation3d(
			cameraPoseFieldRelative.getRotation().getX(),
			-cameraPoseFieldRelative.getRotation().getY(),
			cameraPoseFieldRelative.getRotation().getZ() - robotPoseFieldRelative.getRotation().getRadians()
		);
		// limelight is funny so we invert y-axis

		currentCameraTranslation = new Translation3d(
			cameraPoseFieldRelative.getX() - robotPoseFieldRelative.getX(),
			-(cameraPoseFieldRelative.getY() - robotPoseFieldRelative.getY()),
			cameraPoseFieldRelative.getZ() - tagPoseFieldRelative.getZ() + middleOfTagHeight
		);
	}

	private void sumObjectsValues() {
		// it just creates another object and throws it away , should update the original that is the cause of the change

		translationSum = translationSum.plus(new Translation3d(currentCameraTranslation.getX(), currentCameraTranslation.getY(), currentCameraTranslation.getZ()));
		cosX3DSum += Math.cos(currentCameraRotation.getX());
		sinX3DSum += Math.sin(currentCameraRotation.getX());
		cosY3DSum += Math.cos(-currentCameraRotation.getY());
		sinY3DSum += Math.sin(-currentCameraRotation.getY());
		cosZ3DSum += Math.cos(currentCameraRotation.getZ());
		sinZ3DSum += Math.sin(currentCameraRotation.getZ());
	}

}
