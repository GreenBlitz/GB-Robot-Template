package frc.utils.math;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.*;

public class ObjectDetectionMath {

	public static Pair<Rotation2d, Rotation2d> correctForCameraRoll(Rotation2d yaw, Rotation2d pitch, Pose3d cameraPose) {
		Translation2d yawAndPitch = new Translation2d(yaw.getRadians(), pitch.getRadians());

//		Rotation3d cameraRotation = cameraPose.getRotation().rotateBy(new Rotation3d(0, 0, -cameraPose.getRotation().getZ()));
//		double rollRadians = cameraRotation.rotateBy(new Rotation3d(0, -cameraRotation.getY(), 0)).getX();
		double rollRadians = cameraPose.getRotation().getX();

		yawAndPitch = yawAndPitch.rotateBy(Rotation2d.fromRadians(rollRadians).unaryMinus());
		return new Pair<>(Rotation2d.fromRadians(yawAndPitch.getX()), Rotation2d.fromRadians(yawAndPitch.getY()));
	}

	public static double getCameraRelativeXDistance(Rotation2d cameraRelativePitch, Pose3d cameraPose, double centerOfObjectHeightMeters) {
//		double cameraPitchRadians = cameraPose.getRotation().rotateBy(new Rotation3d(0, 0, -cameraPose.getRotation().getZ())).getY();
		double cameraPitchRadians = cameraPose.getRotation().getY();
		Rotation2d pitch = cameraRelativePitch.plus(Rotation2d.fromRadians(cameraPitchRadians));

		double heightMeters = centerOfObjectHeightMeters - cameraPose.getZ();
		return heightMeters / pitch.getTan();
	}

	public static double getCameraRelativeYDistance(
		Rotation2d cameraRelativeYaw,
		double XDistanceMeters,
		Pose3d cameraPose,
		double centerOfObjectHeightMeters
	) {
		double heightMeters = centerOfObjectHeightMeters - cameraPose.getZ();
		double cameraToObjectXAxisHypotenuseMeters = Math.hypot(XDistanceMeters, heightMeters);

//		tx (represented by cameraRelativeYaw) is flipped (unaryMinus) because of x-axis positive direction conventions
		return cameraRelativeYaw.unaryMinus().getTan() * cameraToObjectXAxisHypotenuseMeters;
	}

	public static Translation2d cameraRelativeToRobotRelative(Translation2d translation, Pose3d cameraPose) {
//		translation = translation.rotateBy(Rotation2d.fromRadians(cameraPose.getRotation().getZ()));
		return new Translation2d(translation.getX() + cameraPose.getX(), translation.getY() + cameraPose.getY());
	}

	public static Translation2d getRobotRelativeTranslation(
		Rotation2d cameraRollRelativeYaw,
		Rotation2d cameraRollRelativePitch,
		Pose3d cameraPose,
		double centerOfObjectHeightMeters
	) {
//		Pair<Rotation2d, Rotation2d> correctedYawAndPitch = correctForCameraRoll(cameraRollRelativeYaw, cameraRollRelativePitch, cameraPose);
		Pair<Rotation2d, Rotation2d> correctedYawAndPitch = new Pair<>(cameraRollRelativeYaw, cameraRollRelativePitch);

		double xDistance = getCameraRelativeXDistance(correctedYawAndPitch.getSecond(), cameraPose, centerOfObjectHeightMeters);
		double yDistance = getCameraRelativeYDistance(correctedYawAndPitch.getFirst(), xDistance, cameraPose, centerOfObjectHeightMeters);
		Translation2d cameraRelativeTranslation = new Translation2d(xDistance, yDistance);
		return cameraRelativeToRobotRelative(cameraRelativeTranslation, cameraPose);
	}

}
