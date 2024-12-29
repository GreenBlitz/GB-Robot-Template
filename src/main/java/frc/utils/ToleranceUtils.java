package frc.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.constants.MathConstants;

public class ToleranceUtils {

	public static boolean isInRange(double value, double maxValue, double minValue, double tolerance) {
		return (minValue - tolerance) <= value && value <= (maxValue + tolerance);
	}

	public static boolean isInRange(double value, double maxValue, double minValue) {
		return isInRange(value, maxValue, minValue, 0);
	}

}
