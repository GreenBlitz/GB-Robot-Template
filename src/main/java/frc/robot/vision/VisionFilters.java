package frc.robot.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.constants.field.Field;
import frc.robot.vision.data.AprilTagVisionData;
import frc.robot.vision.data.LimeLightAprilTagVisionData;
import frc.robot.vision.data.VisionData;
import frc.robot.vision.sources.limelights.LimelightPoseEstimationMethod;
import frc.utils.Filter;
import frc.utils.math.ToleranceMath;

import java.util.Arrays;
import java.util.Optional;
import java.util.function.Supplier;

public class VisionFilters {

	public static Filter<VisionData> isDataFromMegaTag2() {
		return (visionData) -> visionData instanceof LimeLightAprilTagVisionData limelightVisionData
			&& limelightVisionData.getPoseEstimationMethod() == LimelightPoseEstimationMethod.MEGATAG_2;
	}

	public static Filter<VisionData> isRollAtAngle(Rotation2d wantedRoll, Rotation2d rollTolerance) {
		return (visionData) -> ToleranceMath
			.isNearWrapped(wantedRoll, Rotation2d.fromRadians(visionData.getEstimatedPose().getRotation().getX()), rollTolerance);
	}

	public static Filter<VisionData> isPitchAtAngle(Rotation2d wantedPitch, Rotation2d pitchTolerance) {
		return (visionData) -> ToleranceMath
			.isNearWrapped(wantedPitch, Rotation2d.fromRadians(visionData.getEstimatedPose().getRotation().getY()), pitchTolerance);
	}

	public static Filter<VisionData> isYawAtAngle(Supplier<Optional<Rotation2d>> wantedYawSupplier, Rotation2d yawTolerance) {
		return (visionData) -> wantedYawSupplier.get()
			.map(
				wantedAngle -> ToleranceMath
					.isNearWrapped(wantedAngle, Rotation2d.fromRadians(visionData.getEstimatedPose().getRotation().getZ()), yawTolerance)
			)
			.orElse(false);
	}

	public static Filter<VisionData> isYawAtAngleForMegaTag2(Supplier<Optional<Rotation2d>> wantedYawSupplier, Rotation2d yawTolerance) {
		return isDataFromMegaTag2().implies(isYawAtAngle(wantedYawSupplier, yawTolerance));
	}

	public static Filter<VisionData> isYawAngleNotZero() {
		return (visionData) -> visionData.getEstimatedPose().getRotation().getZ() != 0.0;
	}

	public static Filter<VisionData> isYawAngleNotZeroForMegaTag2() {
		return isDataFromMegaTag2().implies(isYawAngleNotZero());
	}

	public static Filter<VisionData> isOnGround(double distanceFromGroundToleranceMeters) {
		return (visionData) -> ToleranceMath.isNear(0, visionData.getEstimatedPose().getZ(), distanceFromGroundToleranceMeters);
	}

	public static Filter<AprilTagVisionData> isAprilTagHeightValid(double aprilTagHeightToleranceMeters) {
		return aprilTagVisionData -> ToleranceMath.isNear(
			VisionUtils.getAprilTagHeightByID(aprilTagVisionData.getTrackedAprilTagId()),
			aprilTagVisionData.getAprilTagHeightMeters(),
			aprilTagHeightToleranceMeters
		);
	}

	public static Filter<VisionData> isXInField(double xToleranceMeters) {
		return (visionData) -> ToleranceMath.isInRange(visionData.getEstimatedPose().getX(), 0, Field.LENGTH_METERS, xToleranceMeters);
	}

	public static Filter<VisionData> isYInField(double yToleranceMeters) {
		return (visionData) -> ToleranceMath.isInRange(visionData.getEstimatedPose().getY(), 0, Field.WIDTH_METERS, yToleranceMeters);
	}

	public static Filter<VisionData> isInField(double positionToleranceMeters) {
		return isXInField(positionToleranceMeters).and(isYInField(positionToleranceMeters));
	}

	public static Filter<AprilTagVisionData> isSeeingTag(int tagID) {
		return (visionData) -> visionData.getTrackedAprilTagId() == tagID;
	}

	public static Filter<AprilTagVisionData> isNotSeeingTags(int... tags) {
		return Filter.orAll(Arrays.stream(tags).mapToObj(VisionFilters::isSeeingTag).toList()).not();
	}

}
