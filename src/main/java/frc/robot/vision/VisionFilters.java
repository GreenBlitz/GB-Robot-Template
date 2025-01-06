package frc.robot.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.constants.field.Field;
import frc.robot.vision.data.AprilTagVisionData;
import frc.robot.vision.data.VisionData;
import frc.utils.Filter;
import frc.utils.math.ToleranceMath;

public class VisionFilters {

	public static Filter<VisionData> isPitchAtAngle(Rotation2d wantedPitch, Rotation2d pitchTolerance) {
		return new Filter<>(
			visionData -> ToleranceMath
				.isNearWrapped(wantedPitch, Rotation2d.fromRadians(visionData.getEstimatedPose().getRotation().getY()), pitchTolerance)
		);
	}

	public static Filter<VisionData> isRollAtAngle(Rotation2d wantedRoll, Rotation2d rollTolerance) {
		return new Filter<>(
			visionData -> ToleranceMath
				.isNearWrapped(wantedRoll, Rotation2d.fromRadians(visionData.getEstimatedPose().getRotation().getX()), rollTolerance)
		);
	}

	public static Filter<VisionData> isOnGround(double distanceFromGroundToleranceMeters) {
		return new Filter<>(visionData -> MathUtil.isNear(0, visionData.getEstimatedPose().getZ(), distanceFromGroundToleranceMeters));
	}

	public static Filter<AprilTagVisionData> isAprilTagHeightValid(double aprilTagRealHeightMeters, double aprilTagHeightToleranceMeters) {
		return new Filter<>(
			aprilTagVisionData -> MathUtil
				.isNear(aprilTagRealHeightMeters, aprilTagVisionData.getAprilTagHeightMeters(), aprilTagHeightToleranceMeters)
		);
	}

	public static Filter<VisionData> isXInField(double xToleranceMeters) {
		return new Filter<>(
			visionData -> ToleranceMath.isInRange(visionData.getEstimatedPose().getX(), 0, Field.LENGTH_METERS, xToleranceMeters)
		);
	}

	public static Filter<VisionData> isYInField(double yToleranceMeters) {
		return new Filter<>(
			visionData -> ToleranceMath.isInRange(visionData.getEstimatedPose().getY(), 0, Field.WIDTH_METERS, yToleranceMeters)
		);
	}

	public static Filter<VisionData> isInField(double positionToleranceMeters) {
		return isXInField(positionToleranceMeters).and(isYInField(positionToleranceMeters));
	}

}
