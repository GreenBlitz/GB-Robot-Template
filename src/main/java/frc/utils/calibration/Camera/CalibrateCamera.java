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
	private AprilTagFields field;
	private final String logPathPrefix;
	private final String cameraName;
	private final int tagID;
	private final double robotXAxisDistanceFromTag;
	private final double middleOfTagHeight;
	private final Pose3d tagPoseFieldRelative;
	private  Pose3d cameraPoseFieldRelative;
	private Pose2d robotPoseFieldRelative;
	private Rotation3d finalCameraRotation;
	private Translation3d finalCameraTranslation;
	private final int neededNumberOfCycles;
	private int currentCycle = 0;

	private double cosX3DSum = 0, sinX3DSum = 0;
	private double cosY3DSum = 0, sinY3DSum = 0;
	private double cosZ3DSum = 0, sinZ3DSum = 0;

	private Translation3d translationSum = new Translation3d();
    private Translation3d currentCameraTrans;
    private  Rotation3d currentCameraRot;

	public CalibrateCamera(
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
		LimelightHelpers.setCameraPose_RobotSpace(cameraName, 0, 0, 0, 0, 0, 0);
		this.tagPoseFieldRelative = AprilTagFieldLayout.loadField(field).getTagPose(tagID).get();
		this.cameraPoseFieldRelative = new Pose3d(LimelightHelpers.getBotPose3d_wpiBlue(cameraName).toMatrix()); // ask dana
		this.robotPoseFieldRelative = new Pose2d(
				//tag must be either 180 or 0 deg to the filed
			tagPoseFieldRelative.getX() - robotXAxisDistanceFromTag,
			tagPoseFieldRelative.getY(),
			FieldMath.transformAngle(tagPoseFieldRelative.getRotation().toRotation2d(), AngleTransform.INVERT)
		);
	}

	private void logFunction() {
		Logger.recordOutput("CameraCalibrateCamera", logPathPrefix+"/solution/Rotations"+finalCameraRotation );
		Logger.recordOutput("CameraCalibration/" + logPathPrefix + "/cam/cameraPoseFieldRelative", cameraPoseFieldRelative);
		Logger.recordOutput("CameraCalibration/" + logPathPrefix + "/robot/robotFieldRelative", robotPoseFieldRelative);
		Logger.recordOutput("CameraCalibration/" + logPathPrefix + "/solution/endTranslation", finalCameraTranslation);
	}

	public void initialize() {
		Logger.recordOutput("CameraCalibration/" + logPathPrefix + "/tag/TagPoseFieldRelative", tagPoseFieldRelative);
	}

	@Override
	public void execute() {
		calculateRobotRelativeCameraPosition();

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
		super.end(interrupted);
	}

	public boolean isFinished() {
		return currentCycle == neededNumberOfCycles;
	}

	private void calculateRobotRelativeCameraPosition() {
		// add option to tags that are not perfectly aligned
		currentCameraRot = new Rotation3d(
			cameraPoseFieldRelative.getRotation().getX(),
			-cameraPoseFieldRelative.getRotation().getY(),
			cameraPoseFieldRelative.getRotation().getZ() - robotPoseFieldRelative.getRotation().getRadians()
		);
		// limelight is funny so we invert y-axis
		currentCameraTrans = new Translation3d(
			cameraPoseFieldRelative.getX() - robotPoseFieldRelative.getX(),
			-(cameraPoseFieldRelative.getY() - robotPoseFieldRelative.getY()),
			cameraPoseFieldRelative.getZ() - tagPoseFieldRelative.getZ() + middleOfTagHeight
		);
		sumObjectsValues();
	}

	private void sumObjectsValues() {
		translationSum.plus(
			new Translation3d(currentCameraTrans.getX(), currentCameraTrans.getY(), currentCameraTrans.getZ()));
		cosX3DSum += Math.cos(currentCameraRot.getX());
		sinX3DSum += Math.sin(currentCameraRot.getX());
		cosY3DSum += Math.cos(-currentCameraRot.getY());
		sinY3DSum += Math.sin(-currentCameraRot.getY());
		cosZ3DSum += Math.cos(currentCameraRot.getZ());
		sinZ3DSum += Math.sin(currentCameraRot.getZ());
	}

}
