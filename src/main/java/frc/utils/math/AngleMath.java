package frc.utils.math;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.constants.MathConstants;

public class AngleMath {

	public static Rotation2d wrappingAbsoluteValue(Rotation2d angle) {
		double rotations = angle.getRotations() % MathConstants.FULL_CIRCLE.getRotations();
		if (rotations < 0) {
			rotations += MathConstants.FULL_CIRCLE.getRotations();
		}
		return Rotation2d.fromRotations(rotations);
	}

	public static Rotation2d getAngleDifferenceWrapped(Rotation2d angle1, Rotation2d angle2) {
		Rotation2d difference = angle1.minus(angle2);
		if (difference.getRadians() > Math.PI) {
			return Rotation2d.fromRadians(MathConstants.FULL_CIRCLE.getRadians() - difference.getRadians());
		}
		return difference;
	}

}
