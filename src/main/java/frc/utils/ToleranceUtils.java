package frc.utils;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.constants.MathConstants;

public class ToleranceUtils {

	public static boolean isNear(Pose2d wantedPose, Pose2d pose, Rotation2d angleTolerance, double translationalToleranceMeters) {
		return isNear(wantedPose.getTranslation(), pose.getTranslation(), translationalToleranceMeters)
			&& isNearWrapped(wantedPose.getRotation(), pose.getRotation(), angleTolerance);
	}

	public static boolean isNear(Translation2d wantedTranslation, Translation2d translation, double toleranceMeters) {
		return translation.getDistance(wantedTranslation) <= toleranceMeters;
	}

	public static boolean isNearWrapped(Rotation2d wantedAngle, Rotation2d angle, Rotation2d tolerance) {
		return MathUtil.isNear(
			wantedAngle.getRadians(),
			angle.getRadians(),
			tolerance.getRadians(),
			MathConstants.HALF_CIRCLE.unaryMinus().getRadians(),
			MathConstants.HALF_CIRCLE.getRadians()
		);
	}

	public static boolean isInRange(double value, double min, double max, double tolerance) {
		return (min - tolerance) <= value && value <= (max + tolerance);
	}

	public static boolean isInRange(double value, double min, double max) {
		return isInRange(value, min, max, 0);
	}

}
