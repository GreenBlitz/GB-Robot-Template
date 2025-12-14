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

	private double sumCosX3D = 0, sumSinX3D = 0;
	private double sumCosY3D = 0, sumSinY3D = 0;
	private double sumCosZ = 0, sumSinZ = 0;
	private double sumCosPose = 0, sumSinPose = 0;
	private double sumTranslationX = 0;
	private double sumTranslationY = 0;
	private double sumTranslationZ = 0;
	private double sumPose2DX = 0;
	private double sumPose2Dy = 0;
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

	public CalibrateCamera(
		AprilTagFields field,
		String PathPrefix,
		String cameraName,
		int tagID,
		double xRobotDistanceFromTag,
		double middleOfTagHeight,
		int numberOfCycles
	) {
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

	public void initialize() {}

	@Override
	public void execute() {
		logCameraPose(this.PathPrefix, this.cameraName, this.tagID, this.xRobotDistanceFromTag, this.middleOfTagHeight);
		logFunction();
		counter += 1;
	}

	@Override
	public void end(boolean interrupted) {
		this.endTranslation = new Translation3d(
			sumTranslationX / this.numberOfCycles,
			sumTranslationY / this.numberOfCycles,
			this.sumTranslationZ / this.numberOfCycles
		);
		this.robotPoseFieldRelative = new Pose2d(
			sumPose2DX / this.numberOfCycles,
			sumPose2Dy / this.numberOfCycles,
			Rotation2d.fromRadians(Math.atan2(sumSinPose / this.numberOfCycles, sumCosPose / this.numberOfCycles))
		);
		this.endRot = new Rotation3d(
			Math.atan2(sumSinX3D / this.numberOfCycles, sumCosX3D / this.numberOfCycles),
			Math.atan2(sumSinY3D / this.numberOfCycles, sumCosY3D / this.numberOfCycles),
			Math.atan2(sumSinZ / this.numberOfCycles, sumCosZ / this.numberOfCycles)
		);
		super.end(interrupted);
	}

	public boolean isFinished() {
		return counter == numberOfCycles;
	}

	private void logCameraPose(String PathPrefix, String cameraName, int tagID, double xRobotDistanceFromTag, double middleOfTagHeight) {
		// add option to tags that are not perfectly aligned
		this.robotPoseFieldRelative = new Pose2d(
			// pose i combined from translation 2d the x and y of the filed and an angle / what angle ?
			tagPoseFieldRelative.getX() - xRobotDistanceFromTag,
			tagPoseFieldRelative.getY(),
			FieldMath.transformAngle(tagPoseFieldRelative.getRotation().toRotation2d(), AngleTransform.INVERT)
		);
		// limelight is funny so we invert pitch
		// represents make average
		this.endRot = new Rotation3d(
			// represents three degrees
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
		sumObjectsValues();
	}

	private void sumObjectsValues() {
		sumTranslationX += cameraPoseFieldRelative.getX() - robotPoseFieldRelative.getX();
		sumTranslationY += -(cameraPoseFieldRelative.getY() - robotPoseFieldRelative.getY());
		sumTranslationZ += cameraPoseFieldRelative.getZ() - tagPoseFieldRelative.getZ() + middleOfTagHeight;
		sumPose2DX += tagPoseFieldRelative.getX() - xRobotDistanceFromTag;
		sumPose2Dy += tagPoseFieldRelative.getY();
		sumCosX3D += Math.cos(cameraPoseFieldRelative.getRotation().getX());
		sumSinX3D += Math.sin(cameraPoseFieldRelative.getRotation().getX());
		sumCosY3D += Math.cos(-cameraPoseFieldRelative.getRotation().getY());
		sumSinY3D += Math.sin(-cameraPoseFieldRelative.getRotation().getY());
		sumCosZ += Math.cos(cameraPoseFieldRelative.getRotation().getZ() - robotPoseFieldRelative.getRotation().getRadians());
		sumSinZ += Math.sin(cameraPoseFieldRelative.getRotation().getZ() - robotPoseFieldRelative.getRotation().getRadians());
		sumCosPose += Math.cos(FieldMath.transformAngle(tagPoseFieldRelative.getRotation().toRotation2d(), AngleTransform.INVERT).getRadians());
		sumSinPose += Math.sin(FieldMath.transformAngle(tagPoseFieldRelative.getRotation().toRotation2d(), AngleTransform.INVERT).getRadians());
	}

}
