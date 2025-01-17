package frc.utils.math;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

import frc.constants.MathConstants;
import frc.constants.field.Field;

public class FieldMath {

	public static Translation2d getRelativeTranslation(Translation2d relativeTo, Translation2d reference) {
		return reference.minus(relativeTo);
	}

	public static Translation2d getRelativeTranslation(Pose2d relativeTo, Translation2d reference) {
		return getRelativeTranslation(relativeTo.getTranslation(), reference).rotateBy(relativeTo.getRotation().unaryMinus());
	}

	/**
	 *
	 * @param angle The angle to mirror
	 * @return The mirrored angle. For example: angle = 45 degrees, return = 135 degrees
	 */
	public static Rotation2d mirrorAngle(Rotation2d angle) {
		return MathConstants.HALF_CIRCLE.minus(angle);
	}

	public static Rotation3d mirrorAngle(Rotation3d rotation) {
		return new Rotation3d(rotation.getX(), -rotation.getY(), rotation.getZ());
	}

	/**
	 *
	 * @param angle The angle to invert
	 * @return The inverted angle. For example: angle = 45 degrees, return = -135 degrees
	 */
	public static Rotation2d invertAngle(Rotation2d angle) {
		return MathConstants.HALF_CIRCLE.plus(angle);
	}

	public static double getMirroredX(double x) {
		return Field.LENGTH_METERS - x;
	}

	public static double getMirroredY(double y) {
		return Field.WIDTH_METERS - y;
	}

	public static Pose2d getMirrored(Pose2d pose2d, boolean mirrorX, boolean mirrorY, boolean invertAngle, boolean mirrorAngle) {
		pose2d = new Pose2d(getMirrored(pose2d.getTranslation(), mirrorX, mirrorY), pose2d.getRotation());
		if (invertAngle) {
			pose2d = new Pose2d(pose2d.getX(), pose2d.getY(), invertAngle(pose2d.getRotation()));
		}
		if (mirrorAngle) {
			pose2d = new Pose2d(pose2d.getX(), pose2d.getY(), mirrorAngle(pose2d.getRotation()));
		}
		return pose2d;
	}

	public static Translation3d getMirrored(Translation3d translation3d, boolean mirrorX, boolean mirrorY) {
		if (mirrorX) {
			translation3d = new Translation3d(getMirroredX(translation3d.getX()), translation3d.getY(), translation3d.getZ());
		}
		if (mirrorY) {
			translation3d = new Translation3d(translation3d.getX(), getMirroredY(translation3d.getY()), translation3d.getZ());
		}
		return translation3d;
	}

	public static Translation2d getMirrored(Translation2d translation2d, boolean mirrorX, boolean mirrorY) {
		if (mirrorY) {
			translation2d = new Translation2d(translation2d.getX(), getMirroredY(translation2d.getY()));
		}
		if (mirrorX) {
			translation2d = new Translation2d(getMirroredX(translation2d.getX()), translation2d.getY());
		}
		return translation2d;
	}

}
