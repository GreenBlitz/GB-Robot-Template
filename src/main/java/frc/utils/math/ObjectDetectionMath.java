package frc.utils.math;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.*;
import org.littletonrobotics.junction.Logger;

public class ObjectDetectionMath {

	public static Pair<Rotation2d, Rotation2d> correctForCameraRoll(Rotation2d yaw, Rotation2d pitch, Pose3d cameraPose) {
		Translation2d yawAndPitch = new Translation2d(yaw.getRadians(), pitch.getRadians());
		double rollRadians = cameraPose.getRotation()
			.rotateBy(new Rotation3d(0, -cameraPose.getRotation().getY(), -cameraPose.getRotation().getZ()))
			.getX();
		Logger.recordOutput("newRoll", Rotation2d.fromRadians(cameraPose.getRotation().getX()).minus(Rotation2d.fromRadians(rollRadians)));
		yawAndPitch = yawAndPitch.rotateBy(Rotation2d.fromRadians(rollRadians).unaryMinus());
		return new Pair<>(Rotation2d.fromRadians(yawAndPitch.getX()), Rotation2d.fromRadians(yawAndPitch.getY()));
	}

	public static double getCameraRelativeXAxisDistance(Rotation2d cameraRelativePitch, Pose3d cameraPose, double centerOfObjectHeightMeters) {
		Rotation2d pitch = cameraRelativePitch.plus(Rotation2d.fromRadians(cameraPose.getRotation().getY()));
		double heightMeters = centerOfObjectHeightMeters - cameraPose.getZ();
		return heightMeters / Math.tan(pitch.getRadians());
	}

	public static double getCameraRelativeYAxisDistance(Rotation2d cameraRelativeYaw, double XAxisDistanceMeters) {
		return (Math.tan(cameraRelativeYaw.getRadians()) * XAxisDistanceMeters);
	}

	public static Translation2d cameraRelativeToRobotRelative(Translation2d translation, Pose3d cameraPose) {
		translation = new Translation2d(translation.getX(), translation.getY());
		translation = translation.rotateBy(Rotation2d.fromRadians(cameraPose.getRotation().getZ()).unaryMinus());
		return new Translation2d(translation.getX() - cameraPose.getX(), translation.getY() - cameraPose.getY());
//		return translation;
	}

	public static Translation2d cameraRollRelativeYawAndPitchToRobotRelativePose(
		Rotation2d cameraRelativeYaw,
		Rotation2d cameraRelativePitch,
		Pose3d cameraPose,
		double centerOfObjectHeightMeters
	) {
		Pair<Rotation2d, Rotation2d> correctedYawAndPitch = correctForCameraRoll(cameraRelativeYaw, cameraRelativePitch, cameraPose);
		Logger.recordOutput("newTX", correctedYawAndPitch.getFirst());
		Logger.recordOutput("newTY", correctedYawAndPitch.getSecond());
		Quaternion cameraRotation = Quaternion.fromRotationVector(cameraPose.getRotation().toVector());
		cameraRotation.times(Quaternion.fromRotationVector(new Rotation3d(-cameraPose.getRotation().getX(), 0, 0).toVector()));
//		Rotation3d newRot = new Rotation3d(cameraRotation.toRotationVector());
		Rotation3d newRot = cameraPose.getRotation().rotateBy(new Rotation3d(-cameraPose.getRotation().getX(), 0, 0));
		Logger.recordOutput("newYaw", Rotation2d.fromRadians(cameraPose.getRotation().getZ()).minus(Rotation2d.fromRadians(newRot.getZ())));
		Logger.recordOutput("newPitch", Rotation2d.fromRadians(cameraPose.getRotation().getY()).minus(Rotation2d.fromRadians(newRot.getY())));
		cameraPose = new Pose3d(cameraPose.getTranslation(), newRot);
		double xDistance = getCameraRelativeXAxisDistance(correctedYawAndPitch.getSecond(), cameraPose, centerOfObjectHeightMeters);
		double yDistance = getCameraRelativeYAxisDistance(correctedYawAndPitch.getFirst(), xDistance);
		Translation2d cameraRelativeTranslation = new Translation2d(xDistance, yDistance);
		return cameraRelativeTranslation;
//		return cameraRelativeToRobotRelative(cameraRelativeTranslation, cameraPose);
	}

}
