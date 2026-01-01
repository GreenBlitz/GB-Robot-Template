package frc.utils.calibration.Camera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.utils.limelight.LimelightHelpers;
import frc.utils.math.AngleTransform;
import frc.utils.math.FieldMath;
import org.littletonrobotics.junction.Logger;

public class CameraPoseCalibration extends Command {

	private final static int NEEDED_NUMBER_OF_CYCLES = 100;
	private final static String LOG_PATH = "/cameraPositionCalibration";
	private final String logPathPrefix;
	private final String cameraName;

	private final double tagCenterHeightMeters;
	private final Pose3d tagPoseFieldRelative;
	private final CameraPositionCalibrationInputsAutoLogged cameraPoseFieldRelativeInputs;
	private Pose2d measuredRobotPoseFieldRelative;

	private Rotation3d finalCameraRotation;
	private Translation3d finalCameraTranslation;
	private int currentCycle;
	private double cosYawRotation3DSum = 0, sinYawRotation3DSum = 0;
	private double cosPitchRotation3DSum = 0, sinPitchRotation3DSum = 0;
	private double cosRollRotation3DSum = 0, sinRollRotation3DSum = 0;

	private Translation3d translationSum;
	private Pose3d currentPose;

	public CameraPoseCalibration(
		String logPathPrefix,
		String cameraName,
		double robotXAxisDistanceFromTag,
		double tagCenterHeightMeters,
		Pose3d tagPoseFieldRelative
	) {
		this.cameraName = cameraName;
		this.tagCenterHeightMeters = tagCenterHeightMeters;
		this.logPathPrefix = logPathPrefix;
		this.tagPoseFieldRelative = tagPoseFieldRelative;
		this.measuredRobotPoseFieldRelative = new Pose2d(
			/**
			 * // tag must be either 180 or 0 deg to the filed // Y difference from the tag is 0
			 **/

			tagPoseFieldRelative.getX() - robotXAxisDistanceFromTag,
			tagPoseFieldRelative.getY(),
			FieldMath.transformAngle(tagPoseFieldRelative.getRotation().toRotation2d(), AngleTransform.INVERT)
		);
		this.cameraPoseFieldRelativeInputs = new CameraPositionCalibrationInputsAutoLogged();
		LimelightHelpers.setCameraPose_RobotSpace(cameraName, 0, 0, 0, 0, 0, 0);
		this.cameraPoseFieldRelativeInputs.cameraPoseFieldRelative = LimelightHelpers.getBotPose3d_wpiBlue(cameraName);
		this.translationSum = new Translation3d();
		this.currentPose = new Pose3d();
	}

	@Override
	public void initialize() {
		Logger.recordOutput(logPathPrefix + LOG_PATH + "/tag/tagPoseFieldRelative", tagPoseFieldRelative);
		Logger.recordOutput(logPathPrefix + LOG_PATH + "/robot/robotPoseFieldRelative", measuredRobotPoseFieldRelative);
	}

	@Override
	public void execute() {
		cameraPoseFieldRelativeInputs.cameraPoseFieldRelative = LimelightHelpers.getBotPose3d_wpiBlue(cameraName);
		Logger.processInputs(logPathPrefix + LOG_PATH + "/camera/currentCameraPose", cameraPoseFieldRelativeInputs);
		currentPose = calculateRobotRelativeCameraPosition();
		sumMeasurementsValues();
		logCurrentCameraPose();
		currentCycle++;
	}

	@Override
	public boolean isFinished() {
		return currentCycle >= NEEDED_NUMBER_OF_CYCLES;
	}

	@Override
	public void end(boolean interrupted) {
		finalCameraTranslation = translationSum.div(currentCycle);
		finalCameraRotation = new Rotation3d(
			Math.atan2(sinYawRotation3DSum / currentCycle, cosYawRotation3DSum / currentCycle),
			Math.atan2(sinPitchRotation3DSum / currentCycle, cosPitchRotation3DSum / currentCycle),
			Math.atan2(sinRollRotation3DSum / currentCycle, cosRollRotation3DSum / currentCycle)
		);
		Pose3d averageCameraPoseFieldRelative = new Pose3d(finalCameraTranslation, finalCameraRotation);
		Logger.recordOutput(logPathPrefix + LOG_PATH + "/solution/endTranslation", finalCameraTranslation);
		Logger.recordOutput(logPathPrefix + LOG_PATH + "/solution/endRotation", finalCameraRotation);
		Logger.recordOutput(logPathPrefix + LOG_PATH + "/solution/endPose", averageCameraPoseFieldRelative);
	}


	private Pose3d calculateRobotRelativeCameraPosition() {
		// limelight is funny so we invert y-axis
		return (new Pose3d(
			cameraPoseFieldRelativeInputs.cameraPoseFieldRelative.getX() - measuredRobotPoseFieldRelative.getX(),
			-(cameraPoseFieldRelativeInputs.cameraPoseFieldRelative.getY() - measuredRobotPoseFieldRelative.getY()),
			cameraPoseFieldRelativeInputs.cameraPoseFieldRelative.getZ() - tagPoseFieldRelative.getZ() + tagCenterHeightMeters,
			new Rotation3d(
				cameraPoseFieldRelativeInputs.cameraPoseFieldRelative.getRotation().getX(),
				-cameraPoseFieldRelativeInputs.cameraPoseFieldRelative.getRotation().getY(),
				cameraPoseFieldRelativeInputs.cameraPoseFieldRelative.getRotation().getZ()
					- measuredRobotPoseFieldRelative.getRotation().getRadians()
			)
		));
	}

	private void sumMeasurementsValues() {
		translationSum = translationSum.plus(currentPose.getTranslation());
		cosYawRotation3DSum += Math.cos(currentPose.getRotation().getX());
		sinYawRotation3DSum += Math.sin(currentPose.getRotation().getX());
		cosPitchRotation3DSum += Math.cos(currentPose.getRotation().getY());
		sinPitchRotation3DSum += Math.sin(currentPose.getRotation().getY());
		cosRollRotation3DSum += Math.cos(currentPose.getRotation().getZ());
		sinRollRotation3DSum += Math.sin(currentPose.getRotation().getZ());
	}

	private void logCurrentCameraPose() {
		Logger.recordOutput(logPathPrefix + LOG_PATH + "/current/currentRotation", currentPose.getRotation());
		Logger.recordOutput(logPathPrefix + LOG_PATH + "/current/currentTranslation", currentPose.getTranslation());
		Logger.recordOutput(logPathPrefix + LOG_PATH + "/current/currentPose", currentPose);
	}

}
