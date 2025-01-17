package frc.utils.math;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

import frc.constants.MathConstants;
import frc.constants.field.Field;
import frc.constants.field.enums.AngleAxis;

public class FieldMath {

	public static Translation2d getRelativeTranslation(Translation2d relativeTo, Translation2d reference) {
		return reference.minus(relativeTo);
	}

	public static Translation2d getRelativeTranslation(Pose2d relativeTo, Translation2d reference) {
		return getRelativeTranslation(relativeTo.getTranslation(), reference).rotateBy(relativeTo.getRotation().unaryMinus());
	}

	/**
	 *
	 * @param angle     The angle
	 * @param angleAxis The axis to mirror
	 * @return The mirrored angle by the axis given
	 */
	public static Rotation2d mirrorAngle(Rotation2d angle, AngleAxis angleAxis) {
		return switch (angleAxis) {
			case KEEP -> angle;
			case MIRROR_X -> MathConstants.HALF_CIRCLE.minus(angle);
			case MIRROR_Y -> MathConstants.FULL_CIRCLE.minus(angle);
			case INVERT -> MathConstants.HALF_CIRCLE.plus(angle);
		};
	}

	public static Rotation3d mirrorAngle(Rotation3d rotation) {
		return new Rotation3d(rotation.getX(), -rotation.getY(), rotation.getZ());
	}

	public static double mirrorX(double x) {
		return Field.LENGTH_METERS - x;
	}

	public static double mirrorY(double y) {
		return Field.WIDTH_METERS - y;
	}

	public static Pose2d mirror(Pose2d pose2d, boolean mirrorX, boolean mirrorY, AngleAxis angleAxis) {
		pose2d = new Pose2d(mirror(pose2d.getTranslation(), mirrorX, mirrorY), pose2d.getRotation());
		return new Pose2d(pose2d.getX(), pose2d.getY(), mirrorAngle(pose2d.getRotation(), angleAxis));
	}

	public static Translation3d mirror(Translation3d translation3d, boolean mirrorX, boolean mirrorY) {
		Translation2d mirrored = mirror(translation3d.toTranslation2d(), mirrorX, mirrorY);
		return new Translation3d(mirrored.getX(), mirrored.getY(), translation3d.getZ());
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
