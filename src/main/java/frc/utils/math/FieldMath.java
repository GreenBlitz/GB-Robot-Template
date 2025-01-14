package frc.utils.math;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

import frc.constants.MathConstants;
import frc.constants.field.Field;

public class FieldMath {

	public static Translation2d getRelativeTranslation(Translation2d relativeTo, Translation2d toRelative) {
		return toRelative.minus(relativeTo);
	}

	public static Translation2d getRelativeTranslation(Pose2d relativeTo, Translation2d toRelative) {
		return getRelativeTranslation(relativeTo.getTranslation(), toRelative).rotateBy(relativeTo.getRotation().unaryMinus());
	}

	public static Rotation2d getMirroredAngle(Rotation2d angle) {
		return MathConstants.HALF_CIRCLE.minus(angle);
	}

	public static Rotation2d getInvertedAngle(Rotation2d angle) {
		return MathConstants.HALF_CIRCLE.plus(angle);
	}

	public static double getMirroredX(double x) {
		return Field.LENGTH_METERS - x;
	}

	public static double getMirroredY(double y) {
		return Field.WIDTH_METERS - y;
	}

	public static Pose2d getMirrored(Pose2d pose2d, boolean mirrorY, boolean mirrorX, boolean invertAngle, boolean mirrorAngle) {
		Pose2d output = pose2d;
		if (mirrorY) {
			output = new Pose2d(output.getX(), getMirroredY(output.getY()), output.getRotation());
		}
		if (mirrorX) {
			output = new Pose2d(getMirroredX(output.getX()), output.getY(), output.getRotation());
		}
		if (invertAngle) {
			output = new Pose2d(output.getX(), output.getY(), getInvertedAngle(output.getRotation()));
		}
		if (mirrorAngle) {
			output = new Pose2d(output.getX(), output.getY(), getMirroredAngle(output.getRotation()));
		}
		return output;
	}

	public static Translation3d getMirrored(Translation3d translation3d, boolean mirrorY, boolean mirrorX, boolean invertZ) {
		Translation3d output = translation3d;
		if (mirrorY) {
			output = new Translation3d(output.getX(), getMirroredY(output.getY()), output.getZ());
		}
		if (mirrorX) {
			output = new Translation3d(getMirroredX(output.getX()), output.getY(), output.getZ());
		}
		if (invertZ) {
			output = new Translation3d(output.getX(), output.getY(), 0 - output.getZ());
		}
		return output;
	}

	public static Translation2d getMirrored(Translation2d translation2d, boolean mirrorY, boolean mirrorX) {
		Translation2d output = translation2d;
		if (mirrorY) {
			output = new Translation2d(output.getX(), getMirroredY(output.getY()));
		}
		if (mirrorX) {
			output = new Translation2d(getMirroredX(output.getX()), output.getY());
		}
		return output;
	}

	public static Rotation3d getMirroredAngle(Rotation3d rotation) {
		return new Rotation3d(rotation.getX(), -rotation.getY(), rotation.getZ());
	}

}
