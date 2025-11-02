package frc.utils.calibration.limelightcalibration;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.function.Supplier;

public class CameraCalibration extends Command {

	private final Supplier<Transform3d> tagToCamera;
	private final Pose3d tagToRobot;
	private List<Pose3d> pose;

	public CameraCalibration(Supplier<Transform3d> tagToCamera, Pose3d tagToRobot) {
		this.tagToCamera = tagToCamera;
		this.tagToRobot = tagToRobot;
		Logger.recordOutput("cameraCalibration", LimelightCalculations.getCameraToRobot(tagToCamera, tagToRobot));
	}

	public void addToPoseList() {
		pose.add(LimelightCalculations.getCameraToRobot(tagToCamera, tagToRobot));
	}

	public Pose3d getAvgPose() {
//		double sumX = 0;
//		double sumY = 0;
//		double sumZ = 0;
//		double cosXRotation = 0;
//		double sinXRotation = 0;
//		double cosYRotation = 0;
//		double sinYRotation = 0;
//		double cosZRotation = 0;
//		double sinZRotation = 0;
		Pose3d sum = new Pose3d();
		for (Pose3d currentPose : pose) {
//			sumX += currentPose.getX();
//			sumY += currentPose.getY();
//			sumZ += currentPose.getZ();
//			cosXRotation += Math.cos(currentPose.getRotation().getX());
//			sinXRotation += Math.sin(currentPose.getRotation().getX());
//			cosYRotation += Math.cos(currentPose.getRotation().getY());
//			sinYRotation += Math.sin(currentPose.getRotation().getY());
//			cosZRotation += Math.cos(currentPose.getRotation().getZ());
//			sinZRotation += Math.sin(currentPose.getRotation().getZ());
//			sum.getRotation().plus(currentPose.getRotation());
			sum = new Pose3d(sum.getTranslation().plus(currentPose.getTranslation()),
LimelightCalculations.getSum(sum.getRotation(), currentPose.getRotation()));
		}
//		return new Pose3d(
//			sumX / pose.size(),
//			sumY / pose.size(),
//			sumZ / pose.size(),
//			new Rotation3d(
//				Math.atan2(sinXRotation, cosXRotation),
//				Math.atan2(sinYRotation, cosYRotation),
//				Math.atan2(sinZRotation, cosZRotation)
//			)
//		);
			return sum.div(pose.size());
	}

	@Override
	public void execute() {
		addToPoseList();
		Logger.recordOutput("/cameraCalibration/cameraToRobot", getAvgPose());
	}

	@Override
	public boolean isFinished() {
		return pose.size() > 100;
	}

}
