package frc.utils.math;

import edu.wpi.first.math.geometry.Rotation2d;

public class AngleMath {

	public static Rotation2d findAngleInRange(Rotation2d angle, Rotation2d rangeMidpoint){
		return Rotation2d.fromRadians(
				angle.getRadians() + 2 * Math.PI * Math.ceil((rangeMidpoint.getRadians() - Math.PI - angle.getRadians()) / (2 * Math.PI))
		);
	}

}
