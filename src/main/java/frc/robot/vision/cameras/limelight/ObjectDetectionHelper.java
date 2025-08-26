package frc.robot.vision.cameras.limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.vision.DetectedObjectObseration;
import frc.robot.vision.DetectedObjectType;

public class ObjectDetectionHelper {

	public static DetectedObjectObseration getDetectedObjectObservation(
		Pose3d cameraPose,
		DetectedObjectType objectType,
		Rotation2d objectToCrosshairYawOffset,
		Rotation2d objectToCrosshairPitchOffset,
		double timestampSeconds
	) {
		double cameraRelativeObjectX = getCameraRelativeObjectX(cameraPose, objectType.getHeightMeters(), objectToCrosshairPitchOffset);

		double cameraRelativeObjectY = getCameraRelativeObjectY(
			cameraPose,
			objectType.getHeightMeters(),
			objectToCrosshairYawOffset,
			cameraRelativeObjectX
		);

		Translation2d cameraRelativeObjectTranslation = new Translation2d(cameraRelativeObjectX, cameraRelativeObjectY);

		Translation2d robotRelativeObjectTranslation = cameraRelativeToRobotRelative(cameraRelativeObjectTranslation, cameraPose.toPose2d());

		return new DetectedObjectObseration(timestampSeconds, robotRelativeObjectTranslation, objectType);
	}

	private static double getCameraRelativeObjectX(
		Pose3d cameraPose,
		double detectedObjectHeightMeters,
		Rotation2d objectToCrosshairPitchOffset
	) {
		return (detectedObjectHeightMeters / 2 - cameraPose.getZ())
			/ objectToCrosshairPitchOffset.plus(Rotation2d.fromRadians(cameraPose.getRotation().getY())).getTan();
	}

	private static double getCameraRelativeObjectY(
		Pose3d cameraPose,
		double detectedObjectHeightMeters,
		Rotation2d objectToCrosshairYawOffset,
		double cameraRelativeObjectX
	) {
		return objectToCrosshairYawOffset.unaryMinus().getTan()
			* Math.hypot(cameraRelativeObjectX, detectedObjectHeightMeters / 2 - cameraPose.getZ());
	}

	private static Translation2d cameraRelativeToRobotRelative(Translation2d cameraRelativeTranslation, Pose2d cameraPose) {
		return cameraRelativeTranslation.rotateBy(cameraPose.getRotation()).plus(cameraPose.getTranslation());
	}

}
