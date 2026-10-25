package frc.utils.math;

import edu.wpi.first.math.geometry.Rotation2d;

public class AngleMath {

	public static double findAngleInRangeRadians(double angle, double rangeMidpoint) {
		return angle + 2 * Math.PI * Math.ceil((rangeMidpoint - Math.PI - angle) / (2 * Math.PI));
	}

	public static Rotation2d findAngleInRange(Rotation2d angle, Rotation2d rangeMidpoint){
		return Rotation2d.fromRadians(findAngleInRangeRadians(angle.getRadians(),rangeMidpoint.getRadians()));
	}

}
