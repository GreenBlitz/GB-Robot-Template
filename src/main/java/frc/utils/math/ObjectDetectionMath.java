package frc.utils.math;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ObjectDetectionMath {

	public static Pair<Rotation2d, Rotation2d> correctForCameraRoll(Rotation2d yaw, Rotation2d pitch, Pose3d cameraPose) {
		Translation2d yawAndPitch = new Translation2d(yaw.getDegrees(), pitch.getDegrees());
		double rollRadians = cameraPose.getRotation().getX();
		yawAndPitch = yawAndPitch.rotateBy(Rotation2d.fromRadians(rollRadians).unaryMinus());
		return new Pair<>(Rotation2d.fromDegrees(yawAndPitch.getX()), Rotation2d.fromDegrees(yawAndPitch.getY()));
	}

	public static double getCameraRelativeXAxisDistance(Rotation2d cameraRelativePitch, Pose3d cameraPose, double centerOfObjectHeightMeters) {
		Rotation2d pitch = cameraRelativePitch.minus(Rotation2d.fromRadians(cameraPose.getRotation().getY()));
		double heightMeters = centerOfObjectHeightMeters - cameraPose.getZ();
		return heightMeters / Math.tan(pitch.getRadians());
	}

	public static double getCameraRelativeYAxisDistance(Rotation2d cameraRelativeYaw, Pose3d cameraPose, double XAxisDistanceMeters) {
		Rotation2d yaw = cameraRelativeYaw;
		return (Math.tan(yaw.getRadians()) * XAxisDistanceMeters);
	}

	public static Translation2d cameraRelativeToRobotRelative(Translation2d translation, Pose3d cameraPose) {
		translation = translation.rotateBy(Rotation2d.fromRadians(cameraPose.getRotation().getZ()));
		return new Translation2d(translation.getX() + cameraPose.getX(), translation.getY() + cameraPose.getY());
	}

	public static Translation2d cameraRollRelativeYawAndPitchToRobotRelativePose(
		Rotation2d cameraRelativeYaw,
		Rotation2d cameraRelativePitch,
		Pose3d cameraPose,
		double centerOfObjectHeightMeters
	) {
		Pair<Rotation2d, Rotation2d> correctedYawAndPitch = correctForCameraRoll(cameraRelativeYaw, cameraRelativePitch, cameraPose);
//		Pair<Rotation2d, Rotation2d> correctedYawAndPitch = new Pair<>(cameraRelativeYaw, cameraRelativePitch);
		double xDistance = getCameraRelativeXAxisDistance(correctedYawAndPitch.getSecond(), cameraPose, centerOfObjectHeightMeters);
		double yDistance = getCameraRelativeYAxisDistance(correctedYawAndPitch.getFirst(), cameraPose, xDistance);
		Translation2d cameraRelativeTranslation = new Translation2d(xDistance, yDistance);
		return cameraRelativeToRobotRelative(cameraRelativeTranslation, cameraPose);
	}

}
