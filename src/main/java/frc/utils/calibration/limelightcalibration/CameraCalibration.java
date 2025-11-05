package frc.utils.calibration.limelightcalibration;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.vision.cameras.limelight.Limelight;
import frc.utils.LimelightHelpers;
import frc.utils.math.AngleMath;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;
import java.util.List;

public class CameraCalibration extends Command {

	private final Limelight limelight;
	private final Pose3d tagToRobot;
	private List<Pose3d> pose;
	private Pose3d sum;
    private double sinXSum;
    private double cosXSum;
    private double sinYSum;
    private double cosYSum;
    private double sinZSum;
    private double cosZSum;

	public CameraCalibration(Limelight limelight, Pose3d tagToRobot) {
		this.limelight = limelight;
		this.tagToRobot = tagToRobot;
		this.sum = new Pose3d();
        this.sinXSum = 0;
        this.cosXSum = 0;
        this.sinYSum = 0;
        this.cosYSum = 0;
        this.sinZSum = 0;
        this.cosZSum = 0;
	}

	public void addToPoseList() {
		pose.add(LimelightCalculations.getCameraToRobot(LimelightHelpers.getTargetPose3d_CameraSpace(limelight.getName()), tagToRobot));
		sum.getTranslation().plus(pose.getLast().getTranslation());
		sum.getRotation().plus(pose.getLast().getRotation());
        sinXSum += Math.sin(pose.getLast().getRotation().getX());
        cosXSum += Math.cos(pose.getLast().getRotation().getX());
        sinYSum += Math.sin(pose.getLast().getRotation().getY());
        cosYSum += Math.cos(pose.getLast().getRotation().getY());
        sinZSum += Math.sin(pose.getLast().getRotation().getZ());
        cosZSum += Math.cos(pose.getLast().getRotation().getZ());
	}

	public Pose3d getAvgPose() {
		return new Pose3d(sum.div(pose.size())., AngleMath.getAngleAverageWrapped(sinXSum, cosXSum, sinYSum, cosYSum, sinZSum, cosZSum, pose.size()));
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
