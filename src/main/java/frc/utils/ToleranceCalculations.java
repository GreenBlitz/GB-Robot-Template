package frc.utils;

import edu.wpi.first.math.geometry.Rotation2d;

public class ToleranceCalculations {

	public static boolean isRotation2dInTolerance(Rotation2d tolerance, double rotationInRadians, double wantedRotationInRadians) {
		return Math.abs(wantedRotationInRadians - rotationInRadians) <= tolerance.getRadians()
			|| Math.abs(wantedRotationInRadians - rotationInRadians) >= Math.toRadians(360) - tolerance.getRadians();
	}

	public static boolean isDoubleInRange(double tolerance, double value, double maximumValue, double minimumValue) {
		return (maximumValue + tolerance) >= value && (minimumValue - tolerance) <= value;
	}

}
