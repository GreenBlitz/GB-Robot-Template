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


	public static Rotation2d transform(Rotation2d angle, AngleTransform angleTransform) {
		return switch (angleTransform) {
			case KEEP -> keepAngle(angle);
			case MIRROR_X -> mirrorXAngle(angle);
			case MIRROR_Y -> mirrorYAngle(angle);
			case INVERT -> invertAngle(angle);
		};
	}

	public static Rotation2d keepAngle(Rotation2d angle) {
		return angle;
	}

	public static Rotation2d mirrorXAngle(Rotation2d angle) {
		return MathConstants.HALF_CIRCLE.minus(angle);
	}

	public static Rotation2d mirrorYAngle(Rotation2d angle) {
		return MathConstants.FULL_CIRCLE.minus(angle);
	}

	public static Rotation2d invertAngle(Rotation2d angle) {
		return MathConstants.HALF_CIRCLE.plus(angle);
	}


	public static Rotation3d mirror(Rotation3d angle) {
		return new Rotation3d(angle.getX(), -angle.getY(), angle.getZ());
	}

	public static double mirrorX(double x) {
		return mirrorValue(Field.LENGTH_METERS, x);
	}

	public static double mirrorY(double y) {
		return mirrorValue(Field.WIDTH_METERS, y);
	}

	public static double mirrorValue(double max, double value) {
		return max - value;
	}

	public static Pose2d mirror(Pose2d pose2d, boolean mirrorX, boolean mirrorY, AngleTransform angleTransform) {
		return new Pose2d(mirror(pose2d.getTranslation(), mirrorX, mirrorY), transform(pose2d.getRotation(), angleTransform));
	}

	public static Translation3d mirror(Translation3d translation3d, boolean mirrorX, boolean mirrorY) {
		return new Translation3d(
			mirrorX ? mirrorX(translation3d.getX()) : translation3d.getX(),
			mirrorY ? mirrorY(translation3d.getY()) : translation3d.getY(),
			translation3d.getZ()
		);
	}

	public static Translation2d mirror(Translation2d translation2d, boolean mirrorX, boolean mirrorY) {
		if (mirrorY) {
			translation2d = new Translation2d(translation2d.getX(), mirrorY(translation2d.getY()));
		}
		if (mirrorX) {
			translation2d = new Translation2d(mirrorX(translation2d.getX()), translation2d.getY());
		}
		return translation2d;
	}

}
