package frc.utils.calibration.limelightcalibration;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.utils.math.AngleMath;


public class LimelightCalculations {

	public static Pose3d getCameraToRobot(Pose3d tagToRobot, Pose3d cameraToTag) {
//		return AngleMath.transformBy(tagToRobot, cameraToTag);
//		cameraToTag = new Pose3d(cameraToTag.getTranslation(), cameraToTag.getRotation().rotateBy(new Rotation3d(Math.PI, Math.PI, Math.PI)));
//		return cameraToTag.transformBy(new Transform3d(new Pose3d(), tagToRobot));
		return tagToRobot.transformBy(new Transform3d(new Pose3d(), cameraToTag));
//		return new Pose3d(tagToRobot.getTranslation().plus(cameraToTag.getTranslation()), tagToRobot.getRotation().plus(cameraToTag.getRotation()));
	}

}
