package frc.utils.calibration.camera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.utils.limelight.LimelightHelpers;
import frc.utils.math.AngleTransform;
import frc.utils.math.FieldMath;
import org.littletonrobotics.junction.Logger;

public class CameraPoseCalibration extends Command {

	private final String logPath;
	private final String cameraName;
	private final int neededNumberOfCycles;

	private final Pose3d tagPoseFieldRelative;
	private final Pose2d expectedRobotPoseFieldRelative;
	private final double tagCenterHeightFromGroundMeters;
	private final Rotation2d robotYawRelativeToTagFacingYaw;

	private final CameraPoseCalibrationInputsAutoLogged cameraPoseCalibrationInputs;

	private double cameraRobotRelativeYawCosSum = 0, cameraRobotRelativeYawSinSum = 0;
	private double cameraRobotRelativePitchCosSum = 0, cameraRobotRelativePitchSinSum = 0;
	private double cameraRobotRelativeRollCosSum = 0, cameraRobotRelativeRollSinSum = 0;

	private Translation3d robotRelativeCameraTranslationSum;
	private Pose3d currentRobotRelativeCameraPose;

	private int currentCycle;

	/**
	 * limelight is funny so we invert the solution's y-axis </br>
	 * Important specifications: tag pose in the field (tagPoseFieldRelative) yaw must be 180 degrees, Y difference from the tag should be 0, the
	 * robot's face that the camera is on should be facing the tag and parallel to it.
	 *
	 * @param cameraName                      - name of the limelight in use
	 * @param neededNumberOfCycles            - number of measurements decided by user
	 * @param robotXAxisDistanceFromTag       - distance of the middle of the robot from the tag, "real life measurement"
	 * @param tagCenterHeightFromGroundMeters - distance of the middle of the tag from the floor, "real life measurement"
	 * @param tagPoseFieldRelative            - position of the tag in the apriltag map on the camera
	 * @param robotYawRelativeToTagFacingYaw  - the robot's yaw relative to its yaw when facing the tag, "real life measurement"
	 *
	 */
	public CameraPoseCalibration(
		String logPathPrefix,
		String cameraName,
		int neededNumberOfCycles,
		double robotXAxisDistanceFromTag,
		double tagCenterHeightFromGroundMeters,
		Pose3d tagPoseFieldRelative,
		Rotation2d robotYawRelativeToTagFacingYaw
	) {
		this.logPath = logPathPrefix + "/cameraPositionCalibration";
		this.cameraName = cameraName;
		this.neededNumberOfCycles = neededNumberOfCycles;

		this.tagPoseFieldRelative = tagPoseFieldRelative;
		this.expectedRobotPoseFieldRelative = new Pose2d(
			tagPoseFieldRelative.getX() - robotXAxisDistanceFromTag,
			tagPoseFieldRelative.getY(),
			FieldMath.transformAngle(tagPoseFieldRelative.getRotation().toRotation2d(), AngleTransform.INVERT)
		);
		this.tagCenterHeightFromGroundMeters = tagCenterHeightFromGroundMeters;
		this.robotYawRelativeToTagFacingYaw = robotYawRelativeToTagFacingYaw;

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
			new Translation3d(
				cameraPoseCalibrationInputs.cameraPoseFieldRelative.getX() - expectedRobotPoseFieldRelative.getX(),
				-(cameraPoseCalibrationInputs.cameraPoseFieldRelative.getY() - expectedRobotPoseFieldRelative.getY()),
				cameraPoseCalibrationInputs.cameraPoseFieldRelative.getZ() - tagPoseFieldRelative.getZ() + tagCenterHeightFromGroundMeters
			),
			new Rotation3d(
				cameraPoseCalibrationInputs.cameraPoseFieldRelative.getRotation().getX(),
				-cameraPoseCalibrationInputs.cameraPoseFieldRelative.getRotation().getY(),
				cameraPoseCalibrationInputs.cameraPoseFieldRelative.getRotation().getZ()
					- expectedRobotPoseFieldRelative.getRotation().getRadians()
			)
		).rotateBy(new Rotation3d(0, 0, robotYawRelativeToTagFacingYaw.unaryMinus().getRadians()));
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
