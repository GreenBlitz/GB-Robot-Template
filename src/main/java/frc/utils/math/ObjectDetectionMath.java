package frc.utils.math;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ObjectDetectionMath {

	public static Translation2d correctForCameraRoll(Rotation2d yaw, Rotation2d pitch, Pose3d cameraPose) {
		Translation2d yawAndPitch = new Translation2d(yaw.getRadians(), pitch.getRadians());
		double roll = cameraPose.getRotation().getX();
		yawAndPitch.rotateBy(Rotation2d.fromRadians(-roll));
		return yawAndPitch;
	}

	public static double getRobotRelativeXAxisDistance(Rotation2d cameraRelativePitch, Pose3d cameraPose, double centerOfObjectHeightMeters) {
		Rotation2d pitch = Rotation2d.fromRadians(cameraPose.getRotation().getY()).plus(cameraRelativePitch);
		double heightMeters = cameraPose.getZ() - centerOfObjectHeightMeters;
		double cameraRelativeDistance = Math.abs(Math.tan(pitch.getRadians()) * heightMeters);
		return cameraRelativeDistance + cameraPose.getX();
	}

	public static double getRobotRelativeYAxisDistance(Rotation2d cameraRelativeYaw, Pose3d cameraPose, double XAxisDistanceMeters) {
		Rotation2d yaw = Rotation2d.fromRadians(cameraPose.getRotation().getZ()).plus(cameraRelativeYaw);
		double cameraRelativeDistance = Math.abs(Math.tan(yaw.getRadians()) * XAxisDistanceMeters);
		return cameraRelativeDistance + cameraPose.getY();
	}

	public static Translation2d cameraRelativeYawAndPitchToRobotRelativePose(
		Rotation2d cameraRelativeYaw,
		Rotation2d cameraRelativePitch,
		Pose3d cameraPose,
		double centerOfObjectHeightMeters
	) {
		Translation2d yawAndPitch = correctForCameraRoll(cameraRelativeYaw, cameraRelativePitch, cameraPose);
		double xDistance = getRobotRelativeXAxisDistance(cameraRelativePitch, cameraPose, centerOfObjectHeightMeters);
		double yDistance = getRobotRelativeYAxisDistance(cameraRelativeYaw, cameraPose, xDistance);
		return new Translation2d(xDistance, yDistance);
	}

}
