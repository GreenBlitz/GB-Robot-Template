package frc.utils.calibration.limelightcalibration;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.vision.cameras.limelight.Limelight;
import frc.utils.LimelightHelpers;
import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.function.Supplier;

public class CameraCalibration extends Command {

	private final Limelight limelight;
	private final Pose3d tagToRobot;
	private List<Pose3d> pose;

	public CameraCalibration(Limelight limelight) {
		this.limelight = limelight;
		this.tagToRobot = limelight.getRobotRelativeCameraPose();
		Logger.recordOutput("cameraCalibration", LimelightCalculations.getCameraToRobot(LimelightHelpers.getTargetPose3d_CameraSpace(limelight.getName()), tagToRobot));
	}

	public void addToPoseList() {
		pose.add(LimelightCalculations.getCameraToRobot(LimelightHelpers.getTargetPose3d_CameraSpace(limelight.getName()), tagToRobot));
	}

	public Pose3d getAvgPose() {
		Pose3d sum = new Pose3d();
		for (Pose3d currentPose : pose) {
			sum.getTranslation().plus(currentPose.getTranslation());
			sum.getRotation().plus(currentPose.getRotation());
		}
		return sum.div(pose.size());
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
