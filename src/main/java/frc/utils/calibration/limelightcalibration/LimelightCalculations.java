package frc.utils.calibration.limelightcalibration;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

import java.util.function.Supplier;

public class LimelightCalculations {

	public static Pose3d getCameraToRobot(Pose3d tagToCamera, Pose3d tagToRobot) {
		return tagToRobot.transformBy(tagToCamera.minus(new Pose3d()).inverse());
	}

	public static Rotation3d add(Rotation3d rotation, Rotation3d other) {
		Rotation2d sumX = new Rotation2d(rotation.getX());
		Rotation2d sumY = new Rotation2d(rotation.getY());
		Rotation2d sumZ = new Rotation2d(rotation.getZ());
		return new Rotation3d(
			sumX.plus(new Rotation2d(other.getX())).getRadians(),
			sumY.plus(new Rotation2d(other.getY())).getRadians(),
			sumZ.plus(new Rotation2d(other.getZ())).getRadians()
		);
	}

}
