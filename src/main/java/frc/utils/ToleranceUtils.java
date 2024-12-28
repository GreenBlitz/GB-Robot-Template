package frc.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.MathConstants;

public class ToleranceUtils {

	public static boolean isRotation2dInTolerance(Rotation2d tolerance, Rotation2d rotation, Rotation2d wantedRotation) {
		return Math.abs(wantedRotation.getRadians() - rotation.getRadians()) <= tolerance.getRadians()
			|| Math.abs(wantedRotation.getRadians() - rotation.getRadians()) >= MathConstants.FULL_CIRCLE.getRadians() - tolerance.getRadians();
	}

	public static boolean isInRange(double value, double tolerance, double maxValue, double minValue) {
		return (maxValue + tolerance) >= value && (minValue - tolerance) <= value;
	}

	public static boolean isInRange(double value, double maxValue, double minValue) {
		return maxValue >= value && minValue <= value;
	}

}
