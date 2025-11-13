package frc.utils.calibration.limelightcalibration;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.vision.cameras.limelight.Limelight;
import frc.utils.AngleUnit;
import frc.utils.LimelightHelpers;
import frc.utils.logger.LoggerUtils;
import frc.utils.math.AngleMath;
import org.littletonrobotics.junction.Logger;


public class CameraCalibration extends Command {

	private final String logPath;
	private final Limelight limelight;
	private final Pose3d tagToRobot;
	private Translation3d translationSum;
	private int posesAmount;
	private double sinXSum;
	private double cosXSum;
	private double sinYSum;
	private double cosYSum;
	private double sinZSum;
	private double cosZSum;

	public CameraCalibration(Limelight limelight, Pose3d tagToRobot, String logPath) {
		this.logPath = logPath;
		this.limelight = limelight;
		this.tagToRobot = tagToRobot;
		this.translationSum = new Translation3d();
		this.posesAmount = 0;
		this.sinXSum = 0;
		this.cosXSum = 0;
		this.sinYSum = 0;
		this.cosYSum = 0;
		this.sinZSum = 0;
		this.cosZSum = 0;
	}

	public void addToPoseList() {
		Pose3d currentPose = LimelightCalculations
			.getCameraToRobot(LimelightHelpers.getTargetPose3d_CameraSpace(limelight.getName()), tagToRobot);
		posesAmount++;
		translationSum = translationSum.plus(currentPose.getTranslation());
		sinXSum += Math.sin(currentPose.getRotation().getX());
		cosXSum += Math.cos(currentPose.getRotation().getX());
		sinYSum += Math.sin(currentPose.getRotation().getY());
		cosYSum += Math.cos(currentPose.getRotation().getY());
		sinZSum += Math.sin(currentPose.getRotation().getZ());
		cosZSum += Math.cos(currentPose.getRotation().getZ());
	}

	public Pose3d getAvgPose() {
		return new Pose3d(
			translationSum.div(posesAmount),
			AngleMath.getAngleAverageWrapped(sinXSum, cosXSum, sinYSum, cosYSum, sinZSum, cosZSum, posesAmount)
		);
	}

	@Override
	public void initialize() {
		posesAmount = 0;
	}

	@Override
	public void execute() {
		addToPoseList();
		Logger.recordOutput(logPath + "/cameraToRobotPose", getAvgPose());
		Logger.recordOutput(logPath + "/cameraToRobotPose2d", getAvgPose().toPose2d());
		LoggerUtils.logRotation3d(getAvgPose().getRotation(), logPath + "/cameraToRobot", AngleUnit.DEGREES);
		Logger.recordOutput(logPath + "/cameraToRobotPoseAmount", posesAmount);
	}

	@Override
	public boolean isFinished() {
		return posesAmount > 100;
	}

}
