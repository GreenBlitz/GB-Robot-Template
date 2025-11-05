package frc.utils.math;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.constants.MathConstants;

import java.util.Arrays;

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

	public static Rotation2d getAngleAverageWrappednooo(Rotation2d angle1, Rotation2d angle2) {
		return Rotation2d.fromRadians(Math.atan2(((angle1.getSin() + angle2.getSin()) / 2), ((angle1.getCos() + angle2.getCos()) / 2)));
	}

	public static Rotation2d getAngleAverageWrapped(Rotation2d... angles) {
		try {
			return Rotation2d.fromRadians(
				Math.atan2(
					Arrays.stream(angles).mapToDouble(Rotation2d::getSin).average().getAsDouble(),
					Arrays.stream(angles).mapToDouble(Rotation2d::getCos).average().getAsDouble()
				)
			);
		} catch (Exception e) {
			return Rotation2d.fromRadians(0);
		}
	}

	public static Rotation2d getAngleAverageWrapped(double sinSum, double cosSum){
		return Rotation2d.fromRadians(Math.atan2(sinSum, cosSum));
	}

	public static Rotation3d getAngleAverageWrapped(Rotation3d... angles){
		return new Rotation3d(getAngleAverageWrapped((Rotation2d) Arrays.stream(angles).map(Rotation3d::getX), (Rotation2d) Arrays.stream(angles).map(Rotation3d::getY), (Rotation2d) Arrays.stream(angles).map(Rotation3d::getZ)));
	}

	public static Rotation3d getAngleAverageWrapped(double sinXSum, double cosXSum, double sinYSum, double cosYSum, double sinZSum, double cosZSum){
		return new Rotation3d(getAngleAverageWrapped(sinXSum, cosXSum).getRadians(), getAngleAverageWrapped(sinYSum, cosYSum).getRadians(), getAngleAverageWrapped(sinZSum, cosZSum).getRadians());
	}

}
