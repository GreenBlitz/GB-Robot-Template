package frc.robot.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.constants.Field;
import frc.robot.vision.data.AprilTagVisionData;
import frc.robot.vision.data.VisionData;
import frc.utils.Filter;
import frc.utils.ToleranceUtils;

public class VisionFilters {

	public static Filter<VisionData> isPitchAtGoal(Rotation2d wantedPitch, Rotation2d pitchTolerance) {
		return new Filter<>(
			visionData -> ToleranceUtils
				.isNearWrapped(Rotation2d.fromRadians(visionData.getEstimatedPose().getRotation().getY()), wantedPitch, pitchTolerance)
		);
	}

	public static Filter<VisionData> isRollAtGoal(Rotation2d wantedRoll, Rotation2d rollTolerance) {
		return new Filter<>(
			visionData -> ToleranceUtils
				.isNearWrapped(Rotation2d.fromRadians(visionData.getEstimatedPose().getRotation().getX()), wantedRoll, rollTolerance)
		);
	}

	public static Filter<VisionData> isOnGround(double distanceFromGroundToleranceMeters) {
		return new Filter<>(visionData -> Math.abs(visionData.getEstimatedPose().getZ()) <= distanceFromGroundToleranceMeters);
	}

	public static Filter<AprilTagVisionData> isAprilTagHeightValid(double aprilTagRealHeightMeters, double aprilTagHeightToleranceMeters) {
		return new Filter<>(
			aprilTagVisionData -> Math.abs(aprilTagVisionData.getAprilTagHeightMeters() - aprilTagRealHeightMeters)
				<= aprilTagHeightToleranceMeters
		);
	}

	public static Filter<VisionData> isXInField(double xToleranceMeters) {
		return new Filter<>(
			visionData -> ToleranceUtils.isInRange(xToleranceMeters, visionData.getEstimatedPose().getX(), Field.LENGTH_METERS, 0)
		);
	}

	public static Filter<VisionData> isYInField(double yToleranceMeters) {
		return new Filter<>(
			visionData -> ToleranceUtils.isInRange(yToleranceMeters, visionData.getEstimatedPose().getY(), Field.WIDTH_METERS, 0)
		);
	}

	public static Filter<VisionData> isInField(double positionToleranceMeters) {
		return isXInField(positionToleranceMeters).and(isYInField(positionToleranceMeters));
	}

}
