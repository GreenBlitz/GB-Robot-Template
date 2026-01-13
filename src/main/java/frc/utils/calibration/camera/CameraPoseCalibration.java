package frc.utils.calibration.camera;

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

	private final int neededNumberOfCycles;
	private final String logPath;
	private final String cameraName;
	private final double tagCenterHeightFromGroundInMeters;
	private final Pose3d tagPoseFieldRelative;
	private final CameraPoseCalibrationInputsAutoLogged cameraPoseCalibrationInputs;
	private final Pose2d expectedRobotPoseFieldRelative;

	private int currentCycle;
	private double cameraRobotRelativeYawCosSum = 0, cameraRobotRelativeYawSinSum = 0;
	private double cameraRobotRelativePitchCosSum = 0, cameraRobotRelativePitchSinSum = 0;
	private double cameraRobotRelativeRollCosSum = 0, cameraRobotRelativeRollSinSum = 0;

	private Translation3d robotRelativeCameraTranslationSum;
	private Pose3d currentRobotRelativeCameraPose;

	/**
	 *
	 * @param cameraName                        - the name of the limelight in use
	 * @param robotXAxisDistanceFromTag         -themiddle of the robot's distance from the tag , IMPORTANT "real life measurement"
	 * @param tagCenterHeightFromGroundInMeters - IMPORTANT !!! the middle of the tag height relative to THE FLOOR , "real life measurement"
	 * @param tagPoseFieldRelative              - synthetic measurement
	 * @param neededNumberOfCycles              - number of measurements decided by user
	 *
	 *                                          IMPORTANT SPECIFICATIONS; limelight is funny so we invert y-axis; tag must be 180 to the field.;
	 *                                          Y difference from the tag is 0.;
	 *
	 */
	public CameraPoseCalibration(
		String logPathPrefix,
		String cameraName,
		double robotXAxisDistanceFromTag,
		double tagCenterHeightFromGroundInMeters,
		Pose3d tagPoseFieldRelative,
		int neededNumberOfCycles
	) {
		this.neededNumberOfCycles = neededNumberOfCycles;
		this.cameraName = cameraName;
		this.tagCenterHeightFromGroundInMeters = tagCenterHeightFromGroundInMeters;
		this.logPath = logPathPrefix + "/cameraPositionCalibration";
		this.tagPoseFieldRelative = tagPoseFieldRelative;
		this.expectedRobotPoseFieldRelative = new Pose2d(
			tagPoseFieldRelative.getX() - robotXAxisDistanceFromTag,
			tagPoseFieldRelative.getY(),
			FieldMath.transformAngle(tagPoseFieldRelative.getRotation().toRotation2d(), AngleTransform.INVERT)
		);
		this.cameraPoseCalibrationInputs = new CameraPoseCalibrationInputsAutoLogged();
		this.robotRelativeCameraTranslationSum = new Translation3d();
		this.currentRobotRelativeCameraPose = new Pose3d();

		LimelightHelpers.setCameraPose_RobotSpace(cameraName, 0, 0, 0, 0, 0, 0);
	}

	@Override
	public void initialize() {
		Logger.recordOutput(logPath + "/tag/tagPoseFieldRelative", tagPoseFieldRelative);
		Logger.recordOutput(logPath + "/robot/robotPoseFieldRelative", expectedRobotPoseFieldRelative);
	}

	@Override
	public void execute() {
		cameraPoseCalibrationInputs.cameraPoseFieldRelative = LimelightHelpers.getBotPose3d_wpiBlue(cameraName);
		Logger.processInputs(logPath, cameraPoseCalibrationInputs);
		currentRobotRelativeCameraPose = calculateRobotRelativeCameraPosition();
		sumMeasurementsValues();
		Logger.recordOutput(logPath + "/current/currentPose", currentRobotRelativeCameraPose);
		currentCycle++;
	}

	@Override
	public boolean isFinished() {
		return currentCycle >= neededNumberOfCycles;
	}

	@Override
	public void end(boolean interrupted) {
		Translation3d finalRobotRelativeCameraTranslation = robotRelativeCameraTranslationSum.div(currentCycle);
		Rotation3d finalRobotRelativeCameraRotation = new Rotation3d(
			Math.atan2(cameraRobotRelativeYawSinSum / currentCycle, cameraRobotRelativeYawCosSum / currentCycle),
			Math.atan2(cameraRobotRelativePitchSinSum / currentCycle, cameraRobotRelativePitchCosSum / currentCycle),
			Math.atan2(cameraRobotRelativeRollSinSum / currentCycle, cameraRobotRelativeRollCosSum / currentCycle)
		);
		Pose3d averageCameraPoseFieldRelative = new Pose3d(finalRobotRelativeCameraTranslation, finalRobotRelativeCameraRotation);
		Logger.recordOutput(logPath + "/solution/endPose", averageCameraPoseFieldRelative);
	}


	private Pose3d calculateRobotRelativeCameraPosition() {
		return new Pose3d(
			cameraPoseCalibrationInputs.cameraPoseFieldRelative.getX() - expectedRobotPoseFieldRelative.getX(),
			-(cameraPoseCalibrationInputs.cameraPoseFieldRelative.getY() - expectedRobotPoseFieldRelative.getY()),
			cameraPoseCalibrationInputs.cameraPoseFieldRelative.getZ() - tagPoseFieldRelative.getZ() + tagCenterHeightFromGroundInMeters,
			new Rotation3d(
				cameraPoseCalibrationInputs.cameraPoseFieldRelative.getRotation().getX(),
				-cameraPoseCalibrationInputs.cameraPoseFieldRelative.getRotation().getY(),
				cameraPoseCalibrationInputs.cameraPoseFieldRelative.getRotation().getZ()
					- expectedRobotPoseFieldRelative.getRotation().getRadians()
			)
		);
	}

	private void sumMeasurementsValues() {
		robotRelativeCameraTranslationSum = robotRelativeCameraTranslationSum.plus(currentRobotRelativeCameraPose.getTranslation());
		cameraRobotRelativeYawCosSum += Math.cos(currentRobotRelativeCameraPose.getRotation().getX());
		cameraRobotRelativeYawSinSum += Math.sin(currentRobotRelativeCameraPose.getRotation().getX());
		cameraRobotRelativePitchCosSum += Math.cos(currentRobotRelativeCameraPose.getRotation().getY());
		cameraRobotRelativePitchSinSum += Math.sin(currentRobotRelativeCameraPose.getRotation().getY());
		cameraRobotRelativeRollCosSum += Math.cos(currentRobotRelativeCameraPose.getRotation().getZ());
		cameraRobotRelativeRollSinSum += Math.sin(currentRobotRelativeCameraPose.getRotation().getZ());
	}

}
