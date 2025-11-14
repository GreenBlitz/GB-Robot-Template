package frc.utils.calibration.limelightcalibration;

import edu.wpi.first.math.geometry.Pose3d;
import frc.utils.math.AngleMath;


public class LimelightCalculations {

	public static Pose3d getCameraToRobot(Pose3d tagToRobot, Pose3d cameraToTag) {
		return AngleMath.transformBy(tagToRobot, cameraToTag);
	}

}
