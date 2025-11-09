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

	public static Rotation2d getAngleAverageWrapped(Rotation2d... angles) {
		try {
			return new Rotation2d(Arrays.stream(angles).mapToDouble(Rotation2d::getSin).average().getAsDouble(),
					Arrays.stream(angles).mapToDouble(Rotation2d::getCos).average().getAsDouble()
			);
		} catch (Exception e) {
			return Rotation2d.fromRadians(0);
		}
	}

	public static Rotation2d getAngleAverageWrapped(double sinSum, double cosSum, int posesAmount) {
		return new Rotation2d(sinSum/posesAmount, cosSum/posesAmount);
	}

	public static Rotation3d getAngleAverageWrapped(Rotation3d... angles) {
		return new Rotation3d(
			getAngleAverageWrapped(
				(Rotation2d) Arrays.stream(angles).map((rotation3d -> Rotation2d.fromRadians(rotation3d.getX()))),
				(Rotation2d) Arrays.stream(angles).map((rotation3d -> Rotation2d.fromRadians(rotation3d.getY()))),
				(Rotation2d) Arrays.stream(angles).map((rotation3d -> Rotation2d.fromRadians(rotation3d.getZ())))
			)
		);
	}

	public static Rotation3d getAngleAverageWrapped(
		double sinXSum,
		double cosXSum,
		double sinYSum,
		double cosYSum,
		double sinZSum,
		double cosZSum,
		int posesAmount
	) {
		return new Rotation3d(
			getAngleAverageWrapped(sinXSum, cosXSum, posesAmount).getRadians(),
			getAngleAverageWrapped(sinYSum, cosYSum, posesAmount).getRadians(),
			getAngleAverageWrapped(sinZSum, cosZSum, posesAmount).getRadians()
		);
	}

}
