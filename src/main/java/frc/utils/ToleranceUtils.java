package frc.utils;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.constants.MathConstants;

public class ToleranceUtils {

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
