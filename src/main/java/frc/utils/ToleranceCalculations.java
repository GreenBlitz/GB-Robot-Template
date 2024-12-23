package frc.utils;

import edu.wpi.first.math.geometry.Rotation2d;

public class ToleranceCalculations {

	public static boolean isRotation2dInTolerance(Rotation2d tolerance, Rotation2d rotation, Rotation2d wantedRotation) {
		return Math.abs(wantedRotation.getRadians() - rotation.getRadians()) <= tolerance.getRadians()
			|| Math.abs(wantedRotation.getRadians() - rotation.getRadians()) >= Math.toRadians(360) - tolerance.getRadians();
	}

	public static boolean isDoubleInRange(double tolerance, double value, double maximumValue, double minimumValue) {
		return (maximumValue + tolerance) >= value && (minimumValue - tolerance) <= value;
	}

}
