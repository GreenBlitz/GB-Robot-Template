package frc.utils.calibration.limelightcalibration;

import edu.wpi.first.math.geometry.Pose3d;


public class LimelightCalculations {

	public static Pose3d getCameraToRobot(Pose3d tagToCamera, Pose3d tagToRobot) {
		return tagToRobot.transformBy(tagToCamera.minus(new Pose3d()).inverse());
	}


}
