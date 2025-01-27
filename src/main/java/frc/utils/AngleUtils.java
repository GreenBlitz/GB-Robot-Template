package frc.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.constants.MathConstants;

public class AngleUtils {

	public static Rotation2d wrappingAbsoluteValue(Rotation2d angle) {
		double rotations = angle.getRotations() % MathConstants.FULL_CIRCLE.getRotations();
		if (rotations < 0) {
			rotations += MathConstants.FULL_CIRCLE.getRotations();
		}
		return Rotation2d.fromRotations(rotations);
	}

}
