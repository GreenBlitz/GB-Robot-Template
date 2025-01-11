package frc.utils.math;

import edu.wpi.first.math.geometry.*;
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

	public static Pose2d getMirrored(Pose2d pose2d, boolean mirrorY, boolean mirrorX, boolean invertAngle) {
		if (mirrorY && mirrorX) {
			return invertAngle
				? new Pose2d(getMirroredX(pose2d.getX()), getMirroredY(pose2d.getY()), getMirroredAngle(pose2d.getRotation()))
				: new Pose2d(getMirroredX(pose2d.getX()), getMirroredY(pose2d.getY()), pose2d.getRotation());
		}
		if (mirrorY) {
			return invertAngle
				? new Pose2d(pose2d.getX(), getMirroredY(pose2d.getY()), getMirroredAngle(pose2d.getRotation()))
				: new Pose2d(pose2d.getX(), getMirroredY(pose2d.getY()), pose2d.getRotation());
		}
		if (mirrorX) {
			return invertAngle
				? new Pose2d(getMirroredX(pose2d.getX()), pose2d.getY(), getMirroredAngle(pose2d.getRotation()))
				: new Pose2d(getMirroredX(pose2d.getX()), pose2d.getY(), pose2d.getRotation());
		}
		return invertAngle
			? new Pose2d(pose2d.getX(), pose2d.getY(), getMirroredAngle(pose2d.getRotation()))
			: new Pose2d(pose2d.getX(), pose2d.getY(), pose2d.getRotation());
	}

	public static Translation3d getMirrored(Translation3d translation3d, boolean invertZ) {
		return invertZ
			? new Translation3d(getMirroredX(translation3d.getX()), getMirroredY(translation3d.getY()), 0 - translation3d.getZ())
			: new Translation3d(getMirroredX(translation3d.getX()), getMirroredY(translation3d.getY()), translation3d.getZ());
	}

	public static Translation2d getMirrored(Translation2d translation2d, boolean invertY) {
		return invertY
			? new Translation2d(getMirroredX(translation2d.getX()), getMirroredY(translation2d.getY()))
			: new Translation2d(getMirroredX(translation2d.getX()), translation2d.getY());
	}

}
