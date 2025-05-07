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

	public static Pose2d rotatePose(Rotation2d rotation, Pose2d toRelative) {
		return toRelative.rotateBy(rotation.unaryMinus());
	}

	public static Rotation2d transformAngle(Rotation2d angle, AngleTransform angleTransform) {
		return switch (angleTransform) {
			case KEEP -> angle;
			case MIRROR_X -> MathConstants.HALF_CIRCLE.minus(angle);
			case MIRROR_Y -> MathConstants.FULL_CIRCLE.minus(angle);
			case INVERT -> MathConstants.HALF_CIRCLE.plus(angle);
		};
	}

	public static Rotation3d transformAngle(
		Rotation3d rotation,
		AngleTransform rollTransform,
		AngleTransform pitchTransform,
		AngleTransform yawTransform
	) {
		double roll = transformAngle(Rotation2d.fromRadians(rotation.getX()), rollTransform).getRadians();
		double pitch = transformAngle(Rotation2d.fromRadians(rotation.getY()), pitchTransform).getRadians();
		double yaw = transformAngle(Rotation2d.fromRadians(rotation.getZ()), yawTransform).getRadians();
		return new Rotation3d(roll, pitch, yaw);
	}

	public static double mirrorX(double x) {
		return Field.LENGTH_METERS - x;
	}

	public static double mirrorY(double y) {
		return Field.WIDTH_METERS - y;
	}

	public static Pose2d mirror(Pose2d pose2d, boolean mirrorX, boolean mirrorY, AngleTransform angleTransform) {
		return new Pose2d(mirror(pose2d.getTranslation(), mirrorX, mirrorY), transformAngle(pose2d.getRotation(), angleTransform));
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
