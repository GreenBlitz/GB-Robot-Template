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

	private double cosX3DSum = 0, sinX3DSum = 0;
	private double cosY3DSum = 0, sinY3DSum = 0;
	private double cosZ3DSum = 0, sinZ3DSum = 0;

	private Translation3d translationSum = new Translation3d();

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
		this.robotPoseFieldRelative = new Pose2d(
			// pose i combined from translation 2d the x and y of the filed and an angle / what angle ?
			tagPoseFieldRelative.getX() - robotXAxisDistanceFromTag,
			tagPoseFieldRelative.getY(),
			FieldMath.transformAngle(tagPoseFieldRelative.getRotation().toRotation2d(), AngleTransform.INVERT)
		);
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
		finalCameraRotation = new Rotation3d(
			cameraPoseFieldRelative.getRotation().getX(),
			-cameraPoseFieldRelative.getRotation().getY(),
			cameraPoseFieldRelative.getRotation().getZ() - robotPoseFieldRelative.getRotation().getRadians()
		);
		// limelight is funny so we invert y-axis
		finalCameraTranslation = new Translation3d(
			cameraPoseFieldRelative.getX() - robotPoseFieldRelative.getX(),
			-(cameraPoseFieldRelative.getY() - robotPoseFieldRelative.getY()),
			cameraPoseFieldRelative.getZ() - tagPoseFieldRelative.getZ() + middleOfTagHeight
		);
		sumObjectsValues();
	}

	private void sumObjectsValues() {
		translationSum.plus(
			new Translation3d(
				cameraPoseFieldRelative.getX() - robotPoseFieldRelative.getX(),
				-(cameraPoseFieldRelative.getY() - robotPoseFieldRelative.getY()),
				cameraPoseFieldRelative.getZ() - tagPoseFieldRelative.getZ() + middleOfTagHeight
			)
		);
		cosX3DSum += Math.cos(cameraPoseFieldRelative.getRotation().getX());
		sinX3DSum += Math.sin(cameraPoseFieldRelative.getRotation().getX());
		cosY3DSum += Math.cos(-cameraPoseFieldRelative.getRotation().getY());
		sinY3DSum += Math.sin(-cameraPoseFieldRelative.getRotation().getY());
		cosZ3DSum += Math.cos(cameraPoseFieldRelative.getRotation().getZ() - robotPoseFieldRelative.getRotation().getRadians());
		sinZ3DSum += Math.sin(cameraPoseFieldRelative.getRotation().getZ() - robotPoseFieldRelative.getRotation().getRadians());
	}

}
