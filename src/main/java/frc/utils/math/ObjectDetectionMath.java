package frc.utils.math;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.*;

public class ObjectDetectionMath {

	public static Pair<Rotation2d, Rotation2d> correctForCameraRoll(Rotation2d yaw, Rotation2d pitch, Pose3d cameraPose) {
		Translation2d yawAndPitch = new Translation2d(yaw.getRadians(), pitch.getRadians());

		Rotation3d cameraRotation = cameraPose.getRotation().rotateBy(new Rotation3d(0, 0, -cameraPose.getRotation().getZ()));
		double rollRadians = cameraRotation.rotateBy(new Rotation3d(0, -cameraRotation.getY(), 0)).getX();

		yawAndPitch = yawAndPitch.rotateBy(Rotation2d.fromRadians(rollRadians).unaryMinus());
		return new Pair<>(Rotation2d.fromRadians(yawAndPitch.getX()), Rotation2d.fromRadians(yawAndPitch.getY()));
	}

	public static double getCameraRelativeXAxisDistance(Rotation2d cameraRelativePitch, Pose3d cameraPose, double centerOfObjectHeightMeters) {
		double cameraPitchRadians = cameraPose.getRotation().rotateBy(new Rotation3d(0, 0, -cameraPose.getRotation().getZ())).getZ();
		Rotation2d pitch = cameraRelativePitch.unaryMinus().plus(Rotation2d.fromRadians(cameraPitchRadians));

		double heightMeters = centerOfObjectHeightMeters - cameraPose.getZ();
		return heightMeters / pitch.getTan();
	}

	public static double getCameraRelativeYAxisDistance(
		Rotation2d cameraRelativeYaw,
		double XAxisDistanceMeters,
		Pose3d cameraPose,
		double centerOfObjectHeightMeters
	) {
		double heightMeters = centerOfObjectHeightMeters - cameraPose.getZ();
		double cameraToObjectXAxisHypotenuseMeters = Math.hypot(XAxisDistanceMeters, heightMeters);
		return (Math.tan(cameraRelativeYaw.unaryMinus().getRadians()) * cameraToObjectXAxisHypotenuseMeters);
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

		double xDistance = getCameraRelativeXAxisDistance(correctedYawAndPitch.getSecond(), cameraPose, centerOfObjectHeightMeters);
		double yDistance = getCameraRelativeYAxisDistance(correctedYawAndPitch.getFirst(), xDistance, cameraPose, centerOfObjectHeightMeters);
		Translation2d cameraRelativeTranslation = new Translation2d(xDistance, yDistance);
		return cameraRelativeToRobotRelative(cameraRelativeTranslation, cameraPose);
	}

}
