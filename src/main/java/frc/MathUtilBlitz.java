package frc;

import edu.wpi.first.units.measure.Angle;

public class MathUtilBlitz {

	private double angleInRadians(Angle angle) {
		return angle.baseUnitMagnitude();
	}

	private double clamp(double val, double min, double max) {
		if (val > min && val < max) {
			return val;
		} else if (val >= max) {
			return max;
		} else {
			return min;
		}
	}

	public static double angleDifferenceRadians(double angle1, double angle2) {
		double baseAngleDiff = (angle1 - angle2) % (2 * Math.PI);
		if (baseAngleDiff > Math.PI) {
			return baseAngleDiff - 2 * Math.PI;
		} else if (baseAngleDiff < -Math.PI) {
			return baseAngleDiff + 2 * Math.PI;
		} else {
			return baseAngleDiff;
		}
	}

}
