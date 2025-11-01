package frc.robot.vision.cameras.limelight;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.vision.DetectedObjectObservation;
import frc.robot.vision.DetectedObjectType;
import frc.utils.math.FieldMath;

public class ObjectDetectionMath {

	public static DetectedObjectObservation getDetectedObjectObservation(
		Pose3d cameraPose,
		DetectedObjectType objectType,
		Rotation2d objectToCrosshairYawOffset,
		Rotation2d objectToCrosshairPitchOffset,
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

	private static double getCameraRelativeObjectX(Pose3d cameraPose, double objectCenterHeightMeters, Rotation2d objectToCrosshairPitchOffset) {
		double objectAndCameraHeightDifferenceMeters = objectCenterHeightMeters - cameraPose.getZ();
		Rotation2d objectAndCameraTotalPitch = objectToCrosshairPitchOffset.plus(Rotation2d.fromRadians(cameraPose.getRotation().getY()));
		return objectAndCameraHeightDifferenceMeters / objectAndCameraTotalPitch.getTan();
	}

	public static Pair<Rotation2d, Rotation2d> convertCornerToCrosshair(Rotation2d txnc, Rotation2d tync, Rotation2d fovX, Rotation2d fovY) {
		return new Pair<>(
				txnc.minus(fovX.div(2)),
				tync.minus(fovY.div(2))
		);
	}

	private static double getCameraRelativeObjectY(
		Pose3d cameraPose,
		double objectCenterHeightMeters,
		Rotation2d objectToCrosshairYawOffset,
		double cameraRelativeObjectXMeters
	) {
		double objectAndCameraHeightDifferenceMeters = objectCenterHeightMeters - cameraPose.getZ();
		double cameraToObjectXAxisHypotenuseMeters = Math.hypot(cameraRelativeObjectXMeters, objectAndCameraHeightDifferenceMeters);
		return objectToCrosshairYawOffset.unaryMinus().getTan() * cameraToObjectXAxisHypotenuseMeters;
	}

}
