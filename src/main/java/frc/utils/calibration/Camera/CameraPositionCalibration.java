package frc.utils.calibration.Camera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.utils.limelight.LimelightHelpers;
import frc.utils.math.AngleTransform;
import frc.utils.math.FieldMath;
import org.littletonrobotics.junction.Logger;

public class CameraPositionCalibration extends Command {

	private final static int NEEDED_NUMBER_OF_CYCLES = 100; // PLACE HOLDER
	private final static String commandLogPath = "/cameraPositionCalibration";
	private final String logPathPrefix;
	private final String cameraName;

	private final double middleOfTagHeight;
	private final Pose3d tagPoseFieldRelative;
	private Pose3d cameraPoseFieldRelative;
	private Pose2d robotPoseFieldRelative;

	private Rotation3d finalCameraRotation;
	private Translation3d finalCameraTranslation;
	private int currentCycle;
	private double cosXRotation3DSum = 0, sinXRotation3DSum = 0;
	private double cosYRotation3DSum = 0, sinYRotation3DSum = 0;
	private double cosZRotation3DSum = 0, sinZRotation3DSum = 0;

	private Translation3d translationSum;
	private Translation3d currentCameraTranslation;
	private Rotation3d currentCameraRotation;

	public CameraPositionCalibration(
		AprilTagFields field,
		String logPathPrefix,
		String cameraName,
		int tagID,
		double robotXAxisDistanceFromTag,
		double middleOfTagHeight
	) {
		this.cameraName = cameraName;
		this.middleOfTagHeight = middleOfTagHeight;
		this.logPathPrefix = logPathPrefix;
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
		Logger.recordOutput(logPathPrefix + commandLogPath + "/solution/currentRotation", currentCameraRotation);
		Logger.recordOutput(logPathPrefix + commandLogPath + "/solution/currentTranslation", currentCameraTranslation);
		Logger.recordOutput(logPathPrefix + commandLogPath + "/cameraPoseFieldRelative", cameraPoseFieldRelative);
	}

	@Override
	public void initialize() {
		Logger.recordOutput(logPathPrefix + commandLogPath + "/tag/TagPoseFieldRelative", tagPoseFieldRelative);
		Logger.recordOutput(logPathPrefix + commandLogPath + "/robot/robotFieldRelative", robotPoseFieldRelative);
	}

	@Override
	public void execute() {
		cameraPoseFieldRelative = LimelightHelpers.getBotPose3d_wpiBlue(cameraName);
		calculateRobotRelativeCameraPosition();
		sumObjectsValues();
		currentCycle += 1;
	}

	@Override
	public void end(boolean interrupted) {
		finalCameraTranslation = translationSum.div(NEEDED_NUMBER_OF_CYCLES);
		finalCameraRotation = new Rotation3d(
			Math.atan2(sinXRotation3DSum / NEEDED_NUMBER_OF_CYCLES, cosXRotation3DSum / NEEDED_NUMBER_OF_CYCLES),
			Math.atan2(sinYRotation3DSum / NEEDED_NUMBER_OF_CYCLES, cosYRotation3DSum / NEEDED_NUMBER_OF_CYCLES),
			Math.atan2(sinZRotation3DSum / NEEDED_NUMBER_OF_CYCLES, cosZRotation3DSum / NEEDED_NUMBER_OF_CYCLES)
		);


		Logger.recordOutput(logPathPrefix + commandLogPath + "/solution/endTranslation", finalCameraTranslation);
		Logger.recordOutput(logPathPrefix + commandLogPath + "/solution/endRotation", finalCameraRotation);
		super.end(interrupted);
	}

	@Override
	public boolean isFinished() {
		return currentCycle >= NEEDED_NUMBER_OF_CYCLES;
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
		translationSum = translationSum
			.plus(new Translation3d(currentCameraTranslation.getX(), currentCameraTranslation.getY(), currentCameraTranslation.getZ()));
		cosXRotation3DSum += Math.cos(currentCameraRotation.getX());
		sinXRotation3DSum += Math.sin(currentCameraRotation.getX());
		cosYRotation3DSum += Math.cos(-currentCameraRotation.getY());
		sinYRotation3DSum += Math.sin(-currentCameraRotation.getY());
		cosZRotation3DSum += Math.cos(currentCameraRotation.getZ());
		sinZRotation3DSum += Math.sin(currentCameraRotation.getZ());
	}

}
