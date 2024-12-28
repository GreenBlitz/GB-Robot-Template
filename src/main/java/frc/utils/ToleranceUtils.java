package frc.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.MathConstants;

public class ToleranceUtils {

	public static boolean wrappedIsNear(Rotation2d tolerance, Rotation2d rotation, Rotation2d wantedRotation) {
		return Math.abs(wantedRotation.getRadians() - rotation.getRadians()) <= tolerance.getRadians()
			|| Math.abs(wantedRotation.getRadians() - rotation.getRadians()) >= MathConstants.FULL_CIRCLE.getRadians() - tolerance.getRadians();
	}

	public static boolean isInRange(double value, double tolerance, double maxValue, double minValue) {
		return (minValue - tolerance) <= value && value <= (maxValue + tolerance);
	}

	public static boolean isInRange(double value, double maxValue, double minValue) {
		return maxValue >= value && minValue <= value;
	}

}
