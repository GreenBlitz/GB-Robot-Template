package frc.utils.math;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ObjectDetectionMath {

	public static Translation2d pixelsToPitchAndYaw(Translation2d pose) {
		double yaw = pose.getX();
		double pitch = pose.getY();
		return new Translation2d(yaw, pitch);
	}

	public static Translation2d correctForCameraRoll(Translation2d pitchAndYaw, Pose3d cameraPose) {
		double roll = cameraPose.getRotation().getX();
		pitchAndYaw.rotateBy(Rotation2d.fromRadians(-roll));
		return pitchAndYaw;
	}

	public static double getXAxisDistance(Rotation2d cameraRelativePitch, Pose3d cameraPose) {
		Rotation2d pitch = Rotation2d.fromRadians(cameraPose.getRotation().getY()).plus(cameraRelativePitch);
		double cameraHeightMeters = cameraPose.getZ();
		return Math.abs(Math.tan(pitch.getRadians()) * cameraHeightMeters);
	}

	public static double getYAxisDistance(Rotation2d cameraRelativeYaw, Pose3d cameraPose, double XAxisDistanceMeters) {
		Rotation2d yaw = Rotation2d.fromRadians(cameraPose.getRotation().getZ()).plus(cameraRelativeYaw);
		return Math.abs(Math.tan(yaw.getRadians()) * XAxisDistanceMeters);
	}

}
