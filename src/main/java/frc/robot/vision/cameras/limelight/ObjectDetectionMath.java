package frc.robot.vision.cameras.limelight;

import org.wpilib.math.geometry.Pose3d;
import frc.utils.costomUnits.Angle2d;
import org.wpilib.math.geometry.Translation2d;
import frc.robot.vision.DetectedObjectObservation;
import frc.robot.vision.DetectedObjectType;
import frc.utils.math.FieldMath;

public class ObjectDetectionMath {

	public static DetectedObjectObservation getDetectedObjectObservation(
		Pose3d cameraPose,
		DetectedObjectType objectType,
		Angle2d objectToCrosshairYawOffset,
		Angle2d objectToCrosshairPitchOffset,
		double timestampSeconds
	) {
		double cameraRelativeObjectXMeters = getCameraRelativeObjectX(
			cameraPose,
			objectType.getCenterHeightFromFloorMeters(),
			objectToCrosshairPitchOffset
		);

		double cameraRelativeObjectYMeters = getCameraRelativeObjectY(
			cameraPose,
			objectType.getCenterHeightFromFloorMeters(),
			objectToCrosshairYawOffset,
			cameraRelativeObjectXMeters
		);

		Translation2d cameraRelativeObjectTranslation = new Translation2d(cameraRelativeObjectXMeters, cameraRelativeObjectYMeters);

		Translation2d robotRelativeObjectTranslation = FieldMath
			.getTranslationRelativeToZero(cameraPose.toPose2d(), cameraRelativeObjectTranslation);

		return new DetectedObjectObservation(objectType, robotRelativeObjectTranslation, timestampSeconds);
	}

	private static double getCameraRelativeObjectX(Pose3d cameraPose, double objectCenterHeightMeters, Angle2d objectToCrosshairPitchOffset) {
		double objectAndCameraHeightDifferenceMeters = objectCenterHeightMeters - cameraPose.getZ();
		Angle2d objectAndCameraTotalPitch = objectToCrosshairPitchOffset.plus(Angle2d.fromRadians(cameraPose.getRotation().getY()));
		return objectAndCameraHeightDifferenceMeters / objectAndCameraTotalPitch.getTan();
	}

	private static double getCameraRelativeObjectY(
		Pose3d cameraPose,
		double objectCenterHeightMeters,
		Angle2d objectToCrosshairYawOffset,
		double cameraRelativeObjectXMeters
	) {
		double objectAndCameraHeightDifferenceMeters = objectCenterHeightMeters - cameraPose.getZ();
		double cameraToObjectXAxisHypotenuseMeters = Math.hypot(cameraRelativeObjectXMeters, objectAndCameraHeightDifferenceMeters);
		return objectToCrosshairYawOffset.unaryMinus().getTan() * cameraToObjectXAxisHypotenuseMeters;
	}

}
