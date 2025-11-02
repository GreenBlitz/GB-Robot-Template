package frc.utils.calibration.limelightcalibration;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

import java.util.function.Supplier;

public class LimelightCalculations {

	public static Pose3d getCameraToRobot(Supplier<Transform3d> tagToCamera, Pose3d tagToRobot) {
		return tagToRobot.transformBy(tagToCamera.get().inverse());
	}

	public static Rotation3d getSum(Rotation3d sum, Rotation3d other){
		Rotation2d sumX = new Rotation2d(sum.getX());
		Rotation2d sumY = new Rotation2d(sum.getY());
		Rotation2d sumZ = new Rotation2d(sum.getZ());
		return new Rotation3d(sumX.plus(new Rotation2d(other.getX())).getRadians(), sumY.plus(new Rotation2d(other.getY())).getRadians(), sumZ.plus(new Rotation2d(other.getZ())).getRadians());
	}

}
