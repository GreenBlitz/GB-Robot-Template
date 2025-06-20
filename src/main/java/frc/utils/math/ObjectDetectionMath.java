package frc.utils.math;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.*;

public class ObjectDetectionMath {

	public static Translation2d findRealSquishedAlgaeCenter(
		Translation2d squishedCenterPixel,
		double algaeHeightToWidthRatio,
		double pictureMaxXPixelValue,
		double pictureMaxYPixelValue
	) {
		if (algaeHeightToWidthRatio > 1) {
			return findRealXSquishedAlgaeCenter(squishedCenterPixel, algaeHeightToWidthRatio, pictureMaxXPixelValue);
		} else {
			return findRealYSquishedAlgaeCenter(squishedCenterPixel, algaeHeightToWidthRatio, pictureMaxYPixelValue);
		}
	}

	private static Translation2d findRealXSquishedAlgaeCenter(
		Translation2d squishedCenterPixel,
		double algaeHeightToWidthRatio,
		double pictureMaxXPixelValue
	) {
		double squishedCenterX = squishedCenterPixel.getX();
		double realCenterX;

		if (squishedCenterX > (pictureMaxXPixelValue / 2)) {
			double objectFrameXLength = 2 * (pictureMaxXPixelValue - squishedCenterX);
			double verticalFrameEdgeSmallestX = pictureMaxXPixelValue - objectFrameXLength;
			realCenterX = ((objectFrameXLength / 2) * algaeHeightToWidthRatio) + verticalFrameEdgeSmallestX;
		} else {
			double objectFrameXLength = 2 * squishedCenterX;
			realCenterX = objectFrameXLength - (squishedCenterX * algaeHeightToWidthRatio);
		}

		return new Translation2d(realCenterX, squishedCenterPixel.getY());
	}

	private static Translation2d findRealYSquishedAlgaeCenter(
		Translation2d squishedCenterPixel,
		double algaeHeightToWidthRatio,
		double pictureMaxYPixelValue
	) {
		double squishedCenterY = squishedCenterPixel.getY();
		double realCenterY;

		if (squishedCenterY > (pictureMaxYPixelValue / 2)) {
			double objectFrameYLength = 2 * (pictureMaxYPixelValue - squishedCenterY);
			double horizontalFrameEdgeSmallestY = pictureMaxYPixelValue - objectFrameYLength;
			realCenterY = ((objectFrameYLength / 2) * (1 / algaeHeightToWidthRatio)) + horizontalFrameEdgeSmallestY;
		} else {
			double objectFrameYLength = 2 * squishedCenterY;
			realCenterY = objectFrameYLength - (squishedCenterY * (1 / algaeHeightToWidthRatio));
		}

		return new Translation2d(squishedCenterPixel.getX(), realCenterY);
	}

	public static boolean isPixelOnEdgeOfPicture(Translation2d pixel, double pictureMaxXPixelValue, double pictureMaxYPixelValue, double tolerance) {
		return ToleranceMath.isInRange(pixel.getX(), 0, pictureMaxXPixelValue, tolerance)
			|| ToleranceMath.isInRange(pixel.getY(), 0, pictureMaxYPixelValue, tolerance);
	}

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
