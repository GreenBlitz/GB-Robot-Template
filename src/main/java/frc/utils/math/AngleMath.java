package frc.utils.math;

public class AngleMath {

	public static double findAngleInRange(double angle, double rangeMidpoint) {
		return angle + 2 * Math.PI * Math.ceil((rangeMidpoint - Math.PI - angle) / (2 * Math.PI));
	}

}
