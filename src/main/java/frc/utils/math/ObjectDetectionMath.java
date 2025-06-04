package frc.utils.math;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ObjectDetectionMath {

	public static Pair<Rotation2d, Rotation2d> correctForCameraRoll(Rotation2d yaw, Rotation2d pitch, Pose3d cameraPose) {
		Translation2d yawAndPitch = new Translation2d(yaw.getRadians(), pitch.getRadians());
		double rollRadians = cameraPose.getRotation().getX();
		yawAndPitch = yawAndPitch.rotateBy(Rotation2d.fromRadians(-rollRadians));
		return new Pair<>(Rotation2d.fromRadians(yawAndPitch.getX()), Rotation2d.fromRadians(yawAndPitch.getY()));
	}

	public static double getCameraRelativeXAxisDistance(Rotation2d cameraRelativePitch, Pose3d cameraPose, double centerOfObjectHeightMeters) {
		Rotation2d pitch = Rotation2d.fromRadians(cameraPose.getRotation().getY()).plus(cameraRelativePitch);
		double heightMeters = centerOfObjectHeightMeters - cameraPose.getZ();
		return Math.tan(pitch.getRadians()) * heightMeters;
	}

	public static double getCameraRelativeYAxisDistance(Rotation2d cameraRelativeYaw, Pose3d cameraPose, double XAxisDistanceMeters) {
		Rotation2d yaw = Rotation2d.fromRadians(cameraPose.getRotation().getZ()).plus(cameraRelativeYaw);
		return -(Math.tan(yaw.getRadians()) * XAxisDistanceMeters);
	}

	public static Translation2d cameraRelativeToRobotRelative(Translation2d translation, Pose3d cameraPose) {
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
		double yDistance = getCameraRelativeYAxisDistance(correctedYawAndPitch.getFirst(), cameraPose, xDistance);
		Translation2d cameraRelativeTranslation = new Translation2d(xDistance, yDistance);
		return cameraRelativeToRobotRelative(cameraRelativeTranslation, cameraPose);
	}

}
