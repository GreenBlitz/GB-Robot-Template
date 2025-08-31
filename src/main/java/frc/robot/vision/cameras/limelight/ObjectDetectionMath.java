package frc.robot.vision.cameras.limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.vision.DetectedObjectObseration;
import frc.robot.vision.DetectedObjectType;
import frc.utils.math.FieldMath;

public class ObjectDetectionMath {

	public static DetectedObjectObseration getDetectedObjectObservation(
		Pose3d cameraPose,
		DetectedObjectType objectType,
		Rotation2d objectToCrosshairYawOffset,
		Rotation2d objectToCrosshairPitchOffset,
		double timestampSeconds
	) {
		double cameraRelativeObjectXMeters = getCameraRelativeObjectX(cameraPose, objectType.getHeightMeters(), objectToCrosshairPitchOffset);

		double cameraRelativeObjectYMeters = getCameraRelativeObjectY(
			cameraPose,
			objectType.getHeightMeters(),
			objectToCrosshairYawOffset,
			cameraRelativeObjectXMeters
		);

		Translation2d cameraRelativeObjectTranslation = new Translation2d(cameraRelativeObjectXMeters, cameraRelativeObjectYMeters);

		Translation2d robotRelativeObjectTranslation = FieldMath.getTranslationRelativeToZero(cameraPose.toPose2d(), cameraRelativeObjectTranslation);

		return new DetectedObjectObseration(objectType, robotRelativeObjectTranslation, timestampSeconds);
	}

	private static double getCameraRelativeObjectX(
		Pose3d cameraPose,
		double detectedObjectHeightMeters,
		Rotation2d objectToCrosshairPitchOffset
	) {
		double objectCenterHeightMeters = detectedObjectHeightMeters / 2;
		double objectAndCameraHeightDifferenceMeters = objectCenterHeightMeters - cameraPose.getZ();
		Rotation2d objectAndCameraTotalPitch = objectToCrosshairPitchOffset.plus(Rotation2d.fromRadians(cameraPose.getRotation().getY()));
		return objectAndCameraHeightDifferenceMeters / objectAndCameraTotalPitch.getTan();
	}

	private static double getCameraRelativeObjectY(
		Pose3d cameraPose,
		double detectedObjectHeightMeters,
		Rotation2d objectToCrosshairYawOffset,
		double cameraRelativeObjectXMeters
	) {
		double objectCenterHeightMeters = detectedObjectHeightMeters / 2;
		double objectAndCameraHeightDifferenceMeters = objectCenterHeightMeters - cameraPose.getZ();
		double cameraToObjectXAxisHypotenuseMeters = Math.hypot(cameraRelativeObjectXMeters, objectAndCameraHeightDifferenceMeters);
		return objectToCrosshairYawOffset.unaryMinus().getTan() * cameraToObjectXAxisHypotenuseMeters;
	}

}
