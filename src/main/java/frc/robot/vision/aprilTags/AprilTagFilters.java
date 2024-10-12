package frc.robot.vision.aprilTags;

import edu.wpi.first.math.geometry.*;
import frc.robot.vision.VisionRawData;

public class AprilTagFilters {

	public static boolean keepLimelightData(
		VisionRawData visionRawData,
		Pose2d currentEstimatedPose,
		double aprilTagHeightMeters,
		AprilTagFiltersTolerances tolerances
	) {
//		return AprilTagFilters.isAprilTagInProperHeight(visionRawData, tolerances.aprilTagHeightToleranceMeters(), aprilTagHeightMeters)
		return AprilTagFilters.isLimelightOutputInTolerance(
			visionRawData,
			currentEstimatedPose,
			tolerances.normalizedPositionTolerance(),
			tolerances.normalizedRotationTolerance()
		)
			&& AprilTagFilters.isRollInTolerance(visionRawData, tolerances.rollTolerance())
			&& AprilTagFilters.isPitchInTolerance(visionRawData, tolerances.pitchTolerance())
			&& AprilTagFilters.isRobotOnGround(visionRawData, tolerances.robotToGroundToleranceMeters())
			&& !AprilTagFilters.isDataTooAmbiguous(visionRawData, tolerances.maximumAmbiguity())
			&& !AprilTagFilters.isLatencyTooHigh(visionRawData, tolerances.maximumLatency());
	}

	protected static boolean isLimelightOutputInTolerance(
		VisionRawData visionRawData,
		Pose2d estimatedPose,
		double normalizedPositionTolerance,
		double normalizedRotationTolerance
	) {
		Pose3d limelightPosition = visionRawData.targetPose();
		Pose3d estimatedPose3d = new Pose3d(
			estimatedPose.getX(),
			estimatedPose.getY(),
			0,
			new Rotation3d(0, 0, estimatedPose.getRotation().getRadians())
		);
		Transform3d transformDifference = limelightPosition.minus(estimatedPose3d);
		Rotation3d rotationDifference = limelightPosition.getRotation().minus(estimatedPose3d.getRotation());
		return transformDifference.getTranslation().getNorm() <= normalizedPositionTolerance
			&& getRotationNorm(rotationDifference) <= normalizedRotationTolerance;
	}

	private static double getRotationNorm(Rotation3d angle) {
		return Math.sqrt(Math.pow(angle.getX(), 2) + Math.pow(angle.getY(), 2) + Math.pow(angle.getZ(), 2));
	}

	protected static boolean isPitchInTolerance(VisionRawData visionData, Rotation2d pitchTolerance) {
		return Math.abs(visionData.targetPose().getRotation().getY()) <= pitchTolerance.getRadians();
	}

	protected static boolean isRollInTolerance(VisionRawData visionData, Rotation2d rollTolerance) {
		return Math.abs(visionData.targetPose().getRotation().getX()) <= rollTolerance.getRadians();
	}

//	protected static boolean
//		isAprilTagInProperHeight(VisionRawData visionRawData, double aprilTagHeightToleranceMeters, double aprilTagHeightMeters) {
//		double aprilTagHeightConfidence = Math.abs(visionRawData.aprilTagHeight() - aprilTagHeightMeters);
//		return aprilTagHeightConfidence <= aprilTagHeightToleranceMeters;
//	}

	protected static boolean isRobotOnGround(VisionRawData visionData, double robotToGroundToleranceMeters) {
		return visionData.targetPose().getZ() <= robotToGroundToleranceMeters;
	}

	protected static boolean isDataTooAmbiguous(VisionRawData visionData, double maximumAmbiguity) {
		return visionData.ambiguity() >= maximumAmbiguity;
	}

	protected static boolean isLatencyTooHigh(VisionRawData visionData, double maximumLatency) {
		return visionData.latency() >= maximumLatency;
	}

}
