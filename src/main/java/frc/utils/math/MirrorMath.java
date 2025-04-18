package frc.utils.math;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.constants.field.Field;
import frc.constants.MathConstants;

public class MirrorMath {

	public static Rotation2d getMirroredAngle(Rotation2d angle) {
		return MathConstants.HALF_CIRCLE.minus(angle);
	}

	public static double getMirroredX(double x) {
		return Field.LENGTH_METERS - x;
	}

	public static double getMirroredY(double y) {
		return Field.WIDTH_METERS - y;
	}

}
