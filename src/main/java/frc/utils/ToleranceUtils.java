package frc.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.constants.MathConstants;

public class ToleranceUtils {

	public static boolean isNearWrapped(Rotation2d rotation, Rotation2d wantedRotation, Rotation2d tolerance) {
		double rotationDifferenceRadians = Math.abs(wantedRotation.getRadians() - rotation.getRadians());
		return rotationDifferenceRadians <= tolerance.getRadians()
			|| rotationDifferenceRadians >= MathConstants.FULL_CIRCLE.getRadians() - tolerance.getRadians();
	}

	public static boolean isInRange(double value, double maxValue, double minValue, double tolerance) {
		return (minValue - tolerance) <= value && value <= (maxValue + tolerance);
	}

	public static boolean isInRange(double value, double maxValue, double minValue) {
		return isInRange(value, maxValue, minValue, 0);
	}

}
