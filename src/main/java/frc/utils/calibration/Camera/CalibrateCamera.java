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
	private double cosPoseSum = 0, sinPoseSum = 0;

	private Translation3d translationSum = new Translation3d();

	private double pose2DXSum = 0;
	private double pose2DYSum = 0;

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
		calculateRobotRelativeCameraPosition();
		currentCycle += 1;
	}

	@Override
	public void end(boolean interrupted) {
		this.finalCameraTranslation = translationSum.div(neededNumberOfCycles);
		this.robotPoseFieldRelative = new Pose2d(
			pose2DXSum / this.neededNumberOfCycles,
			pose2DYSum / this.neededNumberOfCycles,
			Rotation2d.fromRadians(Math.atan2(sinPoseSum / this.neededNumberOfCycles, cosPoseSum / this.neededNumberOfCycles))
		);
		this.finalCameraRotation = new Rotation3d(
			Math.atan2(sinX3DSum / this.neededNumberOfCycles, cosX3DSum / this.neededNumberOfCycles),
			Math.atan2(sinY3DSum / this.neededNumberOfCycles, cosY3DSum / this.neededNumberOfCycles),
			Math.atan2(sinZ3DSum / this.neededNumberOfCycles, cosZ3DSum / this.neededNumberOfCycles)
		);
		super.end(interrupted);
	}

	public boolean isFinished() {
		return currentCycle == neededNumberOfCycles;
	}

	private void calculateRobotRelativeCameraPosition() {
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
		translationSum.plus(
			new Translation3d(
				cameraPoseFieldRelative.getX() - robotPoseFieldRelative.getX(),
				-(cameraPoseFieldRelative.getY() - robotPoseFieldRelative.getY()),
				cameraPoseFieldRelative.getZ() - tagPoseFieldRelative.getZ() + middleOfTagHeight
			)
		);

		pose2DXSum += tagPoseFieldRelative.getX() - robotXAxisDistanceFromTag;
		pose2DYSum += tagPoseFieldRelative.getY();

		cosX3DSum += Math.cos(cameraPoseFieldRelative.getRotation().getX());
		sinX3DSum += Math.sin(cameraPoseFieldRelative.getRotation().getX());
		cosY3DSum += Math.cos(-cameraPoseFieldRelative.getRotation().getY());
		sinY3DSum += Math.sin(-cameraPoseFieldRelative.getRotation().getY());
		cosZ3DSum += Math.cos(cameraPoseFieldRelative.getRotation().getZ() - robotPoseFieldRelative.getRotation().getRadians());
		sinZ3DSum += Math.sin(cameraPoseFieldRelative.getRotation().getZ() - robotPoseFieldRelative.getRotation().getRadians());

		cosPoseSum += Math.cos(FieldMath.transformAngle(tagPoseFieldRelative.getRotation().toRotation2d(), AngleTransform.INVERT).getRadians());
		sinPoseSum += Math.sin(FieldMath.transformAngle(tagPoseFieldRelative.getRotation().toRotation2d(), AngleTransform.INVERT).getRadians());
	}

}
