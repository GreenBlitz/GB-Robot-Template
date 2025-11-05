package frc.utils.calibration.limelightcalibration;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.vision.cameras.limelight.Limelight;
import frc.utils.LimelightHelpers;
import frc.utils.math.AngleMath;
import org.littletonrobotics.junction.Logger;

import java.util.List;

public class CameraCalibration extends Command {

	private final Limelight limelight;
	private final Pose3d tagToRobot;
	private List<Pose3d> pose;
	private Translation3d translationSum;
    private double sinXSum;
    private double cosXSum;
    private double sinYSum;
    private double cosYSum;
    private double sinZSum;
    private double cosZSum;

	public CameraCalibration(Limelight limelight, Pose3d tagToRobot) {
		this.limelight = limelight;
		this.tagToRobot = tagToRobot;
		this.translationSum = new Translation3d();
        this.sinXSum = 0;
        this.cosXSum = 0;
        this.sinYSum = 0;
        this.cosYSum = 0;
        this.sinZSum = 0;
        this.cosZSum = 0;
	}

	public void addToPoseList() {
		pose.add(LimelightCalculations.getCameraToRobot(LimelightHelpers.getTargetPose3d_CameraSpace(limelight.getName()), tagToRobot));
		translationSum.plus(pose.getLast().getTranslation());
        sinXSum += Math.sin(pose.getLast().getRotation().getX());
        cosXSum += Math.cos(pose.getLast().getRotation().getX());
        sinYSum += Math.sin(pose.getLast().getRotation().getY());
        cosYSum += Math.cos(pose.getLast().getRotation().getY());
        sinZSum += Math.sin(pose.getLast().getRotation().getZ());
        cosZSum += Math.cos(pose.getLast().getRotation().getZ());
	}

	public Pose3d getAvgPose() {
		return new Pose3d(translationSum.getX()/pose.size(), translationSum.getY()/ pose.size(), translationSum.getZ()/ pose.size(), AngleMath.getAngleAverageWrapped(sinXSum, cosXSum, sinYSum, cosYSum, sinZSum, cosZSum));
	}

	@Override
	public void execute() {
		addToPoseList();
		Logger.recordOutput("/cameraCalibration/cameraToRobotPose", getAvgPose());
	}

	@Override
	public boolean isFinished() {
		return pose.size() > 100;
	}

}
